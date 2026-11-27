#!/usr/bin/env python3
"""Baseline planner: a simple rule-based (greedy) planner for testing the executor.

    /team/world_state  ->  baseline planner  ->  /team/plan   {"robot1": "HOLD:S1", "robot2": "CROSS:D1"}

It uses the same input and output as every other planner (docs/communication_standards.md),
so it can be swapped for the team's conventional planner or the SMART-LLM planner without
changing anything else. It is a stand-in: small, readable and greedy, not optimal.

Rules, applied to the world state each time it changes (costs alone don't count):
  1. A robot that can reach the exit (it has an EXIT cost) needs no door. Any other robot
     takes the shortest known route (fewest doors) to the exit room that uses only workable
     doors: doors that stay open, or that free robots can open now. With no such route, or
     while the exit is unknown, it heads for the nearest workable door into an UNKNOWN room.
     A crossing in progress continues until the executor reports it done.
  2. Robots are handled in order: robots already crossing first, then by distance to their door.
       - Door kept open without this robot (latches, or a switch someone else holds): CROSS.
         Whoever holds a switch that keeps it open keeps holding it.
       - Otherwise: send the cheapest free robots of the right color to the door's switches
         (HOLD or ACTIVATE; every switch of an all_of door, where one robot may press several
         latches one after the other), never to the switch this robot holds. Once this plan
         keeps the door open, the robot CROSSes (waiting at the door until it opens) or, if it
         holds one of the door's switches, keeps holding it until another switch takes over
         (the handoff).
       - Nobody can open it: the robot stays free and may help another robot instead.
  3. If a robot is still stuck, one robot that could leave explores a door into an UNKNOWN
     room for it. Robots that can reach the exit and that nobody needs go to EXIT; everyone
     else WAITs.
  4. Every plan is checked with the SMART-LLM planner's validator before it is sent (one task
     per robot, one robot per task except EXIT/WAIT, colors, reachable targets, every crossed
     door kept open by the plan). An invalid plan is not sent: the previous plan stays, if it
     is still valid, otherwise every robot WAITs. The plan is re-sent every 5 s and as soon as
     a new subscriber (an executor) connects.

Usage (from the repository root; use the system python3):
    python3 scripts/baseline_planner.py --selftest   # check the rules on the two-room handoff
    python3 scripts/baseline_planner.py              # run (needs /team/world_state)
"""
import json
import sys
import time
from collections import deque
from pathlib import Path

# Every plan is checked with the SMART-LLM planner's validator (the same rules for both planners).
sys.path.append(str(Path(__file__).resolve().parent.parent / "smart_llm_planner"))
try:
    from smart_llm_planner.validator import validate_plan
except ImportError:                           # the package is missing: plans are sent unchecked
    validate_plan = None

WORLD_STATE_TOPIC = "/team/world_state"
PLAN_TOPIC = "/team/plan"
LOG_TOPIC = "/team/planner_log"
REPUBLISH_PERIOD_S = 5.0     # the current plan is re-sent this often, for executors that start late


# ------------------------------------------------------------ planning (no ROS) --

class Planner:
    """One planning call over one world state."""

    def __init__(self, ws):
        self.ws = ws
        self.robots = {r["id"]: r for r in ws["robots"]}
        self.tasks = {t["id"]: t for t in ws["tasks"]}
        self.switches = {s["id"]: s for s in ws["switches"]}
        self.doors = {d["id"]: d for d in ws["doors"]}
        self.costs = ws.get("costs", {})
        self.room_of = {rid: room["id"] for room in ws["rooms"] for rid in room["robots"]}
        self.exit_room = next((room["id"] for room in ws["rooms"] if room["exit"]), None)
        self.graph = {room["id"]: [(d["id"], d["other_room"]) for d in room["doors"]] for room in ws["rooms"]}
        # A robot holds a switch if its task is HOLD:<S> and S is on.
        self.holding = {}
        for rid, r in self.robots.items():
            task = r.get("task") or ""
            s = task[len("HOLD:"):] if task.startswith("HOLD:") else None
            if s in self.switches and self.switches[s]["active"]:
                self.holding[rid] = s
        self.assign, self.reason = {}, {}

    # ---- helpers
    def cost(self, rid, task_id):
        return self.costs.get(rid, {}).get(task_id)

    def can_do(self, rid, task_id):
        t = self.tasks.get(task_id)
        if t is None or self.cost(rid, task_id) is None:
            return False
        return t["needs_color"] is None or t["needs_color"] in self.robots[rid]["colors"]

    def switch_ids(self, door_id):
        open_if = self.doors[door_id]["open_if"]
        if not open_if:                       # None: no switch of this door is known
            return None, []
        (logic, ids), = open_if.items()
        return logic, list(ids)

    def kept_open_by(self, door_id, rid):
        """{switch: holder or None} that keep door_id open for rid without rid's help, or None.

        The same rule as the plan validator: a door stays open through latched switches, or
        hold switches that a robot (other than rid) keeps holding. With "open_if": None (no
        switch of the door is known), only the door's current "open" flag counts.
        """
        logic, ids = self.switch_ids(door_id)
        if not ids:
            return {} if self.doors[door_id]["open"] else None
        on = {}
        for s in ids:
            sw = self.switches[s]
            planned = [h for h, t in self.assign.items() if t in (f"HOLD:{s}", f"ACTIVATE:{s}")]
            holder = next((h for h, held in self.holding.items()
                           if held == s and h != rid and h not in self.assign), None)
            if sw["mode"] == "latch" and sw["active"] or planned:
                on[s] = None
            elif holder is not None:
                on[s] = holder
        if logic == "all_of":
            return on if len(on) == len(ids) else None
        if not on:
            return None
        free = [s for s, h in on.items() if h is None]
        return {free[0]: None} if free else dict([min(on.items())])

    def workable(self, door_id, rid):
        """True if rid could get through door_id: it stays open, or free robots can open it."""
        return self.kept_open_by(door_id, rid) is not None or self.enablers(door_id, rid) is not None

    def next_doors(self, rid):
        """Doors the robot may cross next, best first; [] if it needs none (it can reach the exit).

        The route is the shortest one (fewest doors, over known rooms) that uses only workable
        doors: doors that stay open, or that free robots can open now. It leads to the exit room;
        if there is no such route (or the exit is unknown), to the nearest workable door into an
        UNKNOWN room (exploring). Doors of the robot's own room are tried cheapest first.
        """
        start = self.room_of.get(rid)
        if start is None or start == self.exit_room or self.can_do(rid, "EXIT"):
            return []
        to_exit, explore = self.routes(rid)
        return [to_exit] if to_exit else explore

    def routes(self, rid):
        """(first door to the exit room or None, [first doors toward workable UNKNOWN doors])."""
        start = self.room_of.get(rid)
        first = {start: None}                   # room -> first door on the route there
        queue = deque([start])
        explore = []
        while queue:                            # breadth-first: fewest doors
            room = queue.popleft()
            doors = self.graph.get(room, [])
            if room == start:
                doors = sorted(doors, key=lambda d: (self.cost(rid, f"CROSS:{d[0]}") or 1e9, d[0]))
            for door_id, other in doors:
                if not self.workable(door_id, rid):
                    continue
                step = first[room] or door_id
                if other == "UNKNOWN":
                    if step not in explore:
                        explore.append(step)
                    continue
                if other in first:
                    continue
                first[other] = step
                if other == self.exit_room:
                    return step, explore
                queue.append(other)
        return None, explore

    def enablers(self, door_id, rid):
        """[(robot, task)] that would open door_id for rid, using free robots; None if impossible."""
        logic, ids = self.switch_ids(door_id)
        if not ids:
            return None
        busy = set(self.assign) | {rid}
        options = []
        for s in ids:
            if self.holding.get(rid) == s:
                continue                            # rid holds it already; another switch must take over
            sw = self.switches[s]
            already = any(t in (f"HOLD:{s}", f"ACTIVATE:{s}") for t in self.assign.values())
            if sw["active"] and sw["mode"] == "latch" or already:
                options.append((s, 0.0, []))        # on, or a robot is already sent to it
                continue
            holder = next((h for h, held in self.holding.items() if held == s and h != rid), None)
            if holder is not None and holder not in self.assign:
                options.append((s, 0.0, [(holder, f"HOLD:{s}")]))
                continue
            task = f"ACTIVATE:{s}" if sw["mode"] == "latch" else f"HOLD:{s}"
            # A robot can hold a switch OR cross, not both; a latch it may press itself first.
            pool = [h for h in self.robots if h not in busy and self.can_do(h, task)]
            if sw["mode"] == "latch" and rid not in self.assign and self.can_do(rid, task):
                pool.append(rid)
            if pool:
                h = min(pool, key=lambda h: (self.cost(h, task), h))
                options.append((s, self.cost(h, task), [(h, task)]))
        if logic == "any_of":
            return min(options, key=lambda o: (o[1], o[0]))[2] if options else None
        if len(options) < len(ids):
            return None                       # all_of: some switch can't be served
        chosen, used = [], {}
        for s, _, pairs in options:           # all_of: a different robot for every HOLD
            for h, task in pairs:
                if h in used:
                    if task.startswith("ACTIVATE:") and used[h].startswith("ACTIVATE:"):
                        continue              # one robot presses several latches, one per plan
                    return None
                used[h] = task
                chosen.append((h, task))
        return chosen

    def covered(self, door_id):
        """True if this plan keeps door_id open (the plan validator's rule)."""
        logic, ids = self.switch_ids(door_id)
        if not ids:
            return self.doors[door_id]["open"]
        on = {s for s in ids
              if self.switches[s]["mode"] == "latch" and self.switches[s]["active"]
              or any(t in (f"HOLD:{s}", f"ACTIVATE:{s}") for t in self.assign.values())}
        return on == set(ids) if logic == "all_of" else bool(on)

    def try_door(self, rid, door):
        """Give rid (and its helpers) tasks to get through door; False if that is not possible now."""
        cross = f"CROSS:{door}"
        if cross not in self.tasks or self.cost(rid, cross) is None or cross in self.assign.values():
            return False                          # one robot per door at a time (only EXIT/WAIT are shared)
        kept = self.kept_open_by(door, rid)
        if kept is not None:
            self.assign[rid], self.reason[rid] = cross, f"{door} is open"
            for s, h in kept.items():
                if h is not None:
                    self.assign[h], self.reason[h] = f"HOLD:{s}", f"keeps {door} open for {rid}"
            return True
        helpers = self.enablers(door, rid)
        if helpers is None:
            return False
        for h, task in helpers:
            if h not in self.assign:
                self.assign[h], self.reason[h] = task, f"opens {door} for {rid}"
        if rid in self.assign:
            return True                           # rid presses a latch itself first
        held = self.holding.get(rid)
        _, ids = self.switch_ids(door)
        if held in ids:
            self.assign[rid] = f"HOLD:{held}"
            self.reason[rid] = f"holds {held} until another switch keeps {door} open, then crosses"
        elif self.covered(door):
            self.assign[rid], self.reason[rid] = cross, f"crosses {door} once it opens"
        else:
            self.reason[rid] = f"waits until {door} can be kept open"   # more latches to press first
        return True

    # ---- the plan
    def plan(self):
        doors = {rid: self.next_doors(rid) for rid in self.robots}
        for rid, r in self.robots.items():
            # A crossing ends when the executor reports it done (task back to null), not when the
            # robot's center enters the next room: until then it keeps crossing and its door
            # stays open, so no gate closes on a robot still in the doorway.
            task = r.get("task") or ""
            if task.startswith("CROSS:") and task in self.tasks:
                doors[rid] = [task[len("CROSS:"):]]
        for rid, r in self.robots.items():
            if (r.get("status") or {}).get("type") == "broken":
                self.assign[rid], self.reason[rid] = "WAIT", "broken"

        def order(rid):
            crossing = self.robots[rid].get("task") == f"CROSS:{doors[rid][0]}"
            return (not crossing, self.cost(rid, f"CROSS:{doors[rid][0]}") or 1e9, rid)

        for rid in sorted((r for r in self.robots if doors[r]), key=order):
            if rid not in self.assign:
                any(self.try_door(rid, door) for door in doors[rid])   # first door that works

        stuck = [r for r in self.robots if r not in self.assign and not self.can_do(r, "EXIT")]
        if stuck:
            # Someone can't get anywhere: one robot that could leave explores for them instead
            # (a new room may hold the switch that opens their door).
            for rid in sorted((r for r in self.robots if r not in self.assign and r not in stuck),
                              key=lambda r: (min((self.cost(r, f"CROSS:{d}") or 1e9 for d in self.routes(r)[1]),
                                                 default=1e9), r)):
                if any(self.try_door(rid, door) for door in self.routes(rid)[1]):
                    self.reason[rid] = f"explores for {', '.join(stuck)}"
                    break

        for rid in sorted(self.robots):
            if rid in self.assign:
                continue
            if self.can_do(rid, "EXIT"):
                self.assign[rid], self.reason[rid] = "EXIT", "can reach the exit"
            else:
                self.assign[rid], self.reason[rid] = "WAIT", "nothing useful to do now"
        return ({rid: self.assign[rid] for rid in sorted(self.robots)},
                {rid: self.reason[rid] for rid in sorted(self.robots)})


def plan(ws):
    """World state -> ({robot: task id or "WAIT"}, {robot: reason})."""
    return Planner(ws).plan()


def checked_plan(ws, previous=None, planner=plan):
    """planner(ws), checked with the SMART-LLM plan validator before it is sent.

    Returns (assignments, reasons, errors). If the new plan breaks a rule, `errors` says why and
    the previous plan is kept if it is still valid; otherwise every robot WAITs.
    """
    assignments, reasons = planner(ws)
    if validate_plan is None:
        return assignments, reasons, []
    errors = validate_plan(assignments, ws)
    if not errors:
        return assignments, reasons, []
    if previous is not None and not validate_plan(previous, ws):
        return dict(previous), {rid: "previous plan kept (the new one is invalid)" for rid in previous}, errors
    robots = sorted(r["id"] for r in ws["robots"])
    return {rid: "WAIT" for rid in robots}, {rid: "the new plan is invalid" for rid in robots}, errors


def structure_key(ws):
    """The world state without the costs: planning is redone only when this changes."""
    return json.dumps({k: v for k, v in ws.items() if k != "costs"}, sort_keys=True)


# ------------------------------------------------------------ self-test (no ROS) --

def selftest():
    import world_state as W
    sc = W.TEST_SCENARIO
    bad = 0

    def state(known, xy, on, opened, tasks):
        info = {rid: {"task": t} for rid, t in tasks.items()}
        return W.build_world_state(sc, set(known), xy, on, opened, info)

    def check(name, ws, want):
        nonlocal bad
        got, why = plan(ws)
        ok = got == want
        bad += not ok
        print(f"  {'PASS' if ok else 'FAIL'}  {name}: {got}" + ("" if ok else f"\n        want {want}; {why}"))

    start = {"robot1": (-4.8, -0.65), "robot2": (-4.8, 0.65)}
    print("two-room handoff (S1 red hold in 1,1; S2 blue hold in 1,2; D1 any_of):")
    check("t = 0: red holds S1, blue explores through D1",
          state({"1,1"}, start, {"S1": False, "S2": False}, {"D1": False}, {}),
          {"robot1": "HOLD:S1", "robot2": "CROSS:D1"})
    on_s1 = {"robot1": (-1.1, -1.45), "robot2": (-0.9, 0.0)}
    check("S1 held: blue keeps crossing, red keeps holding",
          state({"1,1"}, on_s1, {"S1": True, "S2": False}, {"D1": True},
                {"robot1": "HOLD:S1", "robot2": "CROSS:D1"}),
          {"robot1": "HOLD:S1", "robot2": "CROSS:D1"})
    crossed = {"robot1": (-1.1, -1.45), "robot2": (0.9, 0.0)}
    check("blue crossed: blue holds S2, red keeps S1 until S2 is on",
          state({"1,1", "1,2"}, crossed, {"S1": True, "S2": False}, {"D1": True},
                {"robot1": "HOLD:S1", "robot2": None}),
          {"robot1": "HOLD:S1", "robot2": "HOLD:S2"})
    both_on = {"robot1": (-1.1, -1.45), "robot2": (1.1, 1.45)}
    check("S2 on: red crosses, blue keeps S2",
          state({"1,1", "1,2"}, both_on, {"S1": True, "S2": True}, {"D1": True},
                {"robot1": "HOLD:S1", "robot2": "HOLD:S2"}),
          {"robot1": "CROSS:D1", "robot2": "HOLD:S2"})
    leaving = {"robot1": (-0.6, -0.3), "robot2": (1.1, 1.45)}
    check("red left S1 (S1 off): red keeps crossing, blue keeps S2",
          state({"1,1", "1,2"}, leaving, {"S1": False, "S2": True}, {"D1": True},
                {"robot1": "CROSS:D1", "robot2": "HOLD:S2"}),
          {"robot1": "CROSS:D1", "robot2": "HOLD:S2"})
    doorway = {"robot1": (0.3, 0.0), "robot2": (1.1, 1.45)}
    check("red just past the door line, still crossing: blue keeps holding S2",
          state({"1,1", "1,2"}, doorway, {"S1": False, "S2": True}, {"D1": True},
                {"robot1": "CROSS:D1", "robot2": "HOLD:S2"}),
          {"robot1": "CROSS:D1", "robot2": "HOLD:S2"})
    through = {"robot1": (0.9, 0.0), "robot2": (1.1, 1.45)}
    check("both in the exit room: both EXIT",
          state({"1,1", "1,2"}, through, {"S1": False, "S2": True}, {"D1": True},
                {"robot1": None, "robot2": "HOLD:S2"}),
          {"robot1": "EXIT", "robot2": "EXIT"})

    print("other cases:")
    blue_only = state({"1,1"}, {"robot1": (-4.8, -0.65), "robot2": (-4.8, 0.65)},
                      {"S1": False, "S2": False}, {"D1": False}, {})
    blue_only["robots"][0]["status"] = {"type": "broken", "repair_time": 10, "waiting_for": None}
    check("red robot broken: nobody can open D1, blue waits", blue_only,
          {"robot1": "WAIT", "robot2": "WAIT"})
    sc_none = {**sc, "doors": [{**sc["doors"][0], "logic": "all_of", "requires": ["S2"]}]}
    ws_none = W.build_world_state(sc_none, {"1,1"}, start, {"S1": False, "S2": False}, {"D1": False})
    check("door with open_if None and closed: both wait", ws_none, {"robot1": "WAIT", "robot2": "WAIT"})
    ws_open = W.build_world_state(sc_none, {"1,1"}, start, {"S1": False, "S2": False}, {"D1": True})
    check("door with open_if None but open: one robot crosses at a time", ws_open,
          {"robot1": "CROSS:D1", "robot2": "WAIT"})

    print("exploring: the cheapest door can't be opened, another one can:")
    sc3 = {"rooms": [{"id": "1,1", "bounds": [-6.0, -2.5, 0.0, 2.5]}, {"id": "1,2", "bounds": [0.0, -2.5, 6.0, 2.5]},
                     {"id": "2,1", "bounds": [-6.0, -7.5, 0.0, -2.5]}],
           "robots": [{"id": "robot1", "colors": ["red"]}, {"id": "robot2", "colors": ["blue"]}],
           "switches": [{"id": "S1", "color": "red", "mode": "hold", "pose": [-3.0, 1.5, 0.0]},
                        {"id": "S9", "color": "blue", "mode": "hold", "pose": [3.0, 1.5, 0.0]}],
           "doors": [{"id": "D1", "logic": "all_of", "requires": ["S9"], "pose": [0.0, 0.0, 0.0],
                      "connects": ["1,1", "1,2"]},
                     {"id": "D2", "logic": "any_of", "requires": ["S1"], "pose": [-4.5, -2.5, 0.0],
                      "connects": ["1,1", "2,1"]}],
           "exit": {"id": "EXIT", "pose": [-3.0, -5.0, 0.0]}}
    near_d1 = {"robot1": (-1.0, -0.6), "robot2": (-1.0, 0.6)}
    check("D1 (cheapest, its switch not found) is skipped: red holds S1, blue crosses D2",
          W.build_world_state(sc3, {"1,1"}, near_d1, {"S1": False, "S9": False}, {"D1": False, "D2": False}),
          {"robot1": "HOLD:S1", "robot2": "CROSS:D2"})

    print("a helper is never sent to the switch the crossing robot holds:")
    sc_e4 = {**sc, "switches": [sc["switches"][0]],
             "doors": [{**sc["doors"][0], "requires": ["S1"]}],
             "robots": sc["robots"] + [{"id": "robot3", "colors": ["red"]}]}
    xy_e4 = {"robot1": (-1.1, -1.45), "robot2": (3.0, 0.0), "robot3": (-3.0, 0.0)}
    ws_e4 = W.build_world_state(sc_e4, {"1,1", "1,2"}, xy_e4, {"S1": True}, {"D1": True},
                                {"robot1": {"task": "HOLD:S1"}})
    check("red robot1 holds S1: red robot3 crosses, robot1 keeps holding, robot2 exits", ws_e4,
          {"robot1": "HOLD:S1", "robot2": "EXIT", "robot3": "CROSS:D1"})

    print("one robot presses the two latches of an all_of door, one at a time:")
    sc_latch = {**sc, "switches": [dict(sc["switches"][0], mode="latch"),
                                   {"id": "S3", "color": "red", "mode": "latch", "pose": [-3.0, 1.5, 0.0]}],
                "doors": [{**sc["doors"][0], "logic": "all_of", "requires": ["S1", "S3"]}]}
    check("first latch S1; blue waits (D1 not yet kept open)",
          W.build_world_state(sc_latch, {"1,1"}, start, {"S1": False, "S3": False}, {"D1": False}),
          {"robot1": "ACTIVATE:S1", "robot2": "WAIT"})
    check("S1 on: second latch S3",
          W.build_world_state(sc_latch, {"1,1"}, {"robot1": (-1.1, -1.45), "robot2": (-4.8, 0.65)},
                              {"S1": True, "S3": False}, {"D1": False}),
          {"robot1": "ACTIVATE:S3", "robot2": "CROSS:D1"})

    print("every plan is checked with the SMART-LLM validator:")
    ws_t0 = state({"1,1"}, start, {"S1": False, "S2": False}, {"D1": False}, {})
    invalid = lambda ws: ({"robot1": "HOLD:S1", "robot2": "HOLD:S1"}, {})
    good = {"robot1": "HOLD:S1", "robot2": "CROSS:D1"}
    for name, previous, want in (("invalid new plan: the previous valid plan is kept", good, good),
                                 ("invalid new plan, no previous plan: everyone waits", None,
                                  {"robot1": "WAIT", "robot2": "WAIT"})):
        got, _, errors = checked_plan(ws_t0, previous, planner=invalid)
        ok = got == want and bool(errors) and validate_plan is not None
        bad += not ok
        print(f"  {'PASS' if ok else 'FAIL'}  {name}: {got}" + ("" if ok else f"; errors {errors}"))

    print("ALL PASS" if bad == 0 else f"{bad} FAILED")
    return bad == 0


# ----------------------------------------------------------------- ROS 2 node --

def run():
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("baseline_planner")
    t0 = time.time()
    log = lambda text: print(f"[{time.time() - t0:7.2f}s] {text}", flush=True)
    plan_pub = node.create_publisher(String, PLAN_TOPIC, 10)    # volatile, like every planner
    log_pub = node.create_publisher(String, LOG_TOPIC, 10)
    current = {"key": None, "plan": None, "sent": 0.0, "subscribers": 0}
    if validate_plan is None:
        log("SMART-LLM validator not found (smart_llm_planner/): plans are sent unchecked")

    def send():
        plan_pub.publish(String(data=json.dumps(current["plan"])))
        current["sent"] = time.time()

    def on_world_state(msg):
        try:
            ws = json.loads(msg.data)
        except json.JSONDecodeError as e:
            log(f"ignoring a world state that is not JSON: {e}")
            return
        key = structure_key(ws)
        if key == current["key"]:
            return
        current["key"] = key
        try:
            assignments, reasons, errors = checked_plan(ws, current["plan"])
        except (KeyError, TypeError, ValueError) as e:
            log(f"could not plan on this world state ({type(e).__name__}: {e}); keeping the last plan")
            return
        if errors:
            log(f"the new plan breaks the plan rules ({'; '.join(errors)}); sending {assignments}")
        changed = assignments != current["plan"]
        current["plan"] = assignments
        send()
        log_pub.publish(String(data=json.dumps(
            {"planner": "baseline", "time": time.time(), "assignments": assignments, "reasons": reasons,
             "errors": errors})))
        if changed:
            log("plan: " + ", ".join(f"{rid} {task} ({reasons[rid]})" for rid, task in assignments.items()))

    def on_timer():
        if current["plan"] is None:
            return
        subscribers = plan_pub.get_subscription_count()
        new_subscriber = subscribers > current["subscribers"]
        current["subscribers"] = subscribers
        if new_subscriber or time.time() - current["sent"] >= REPUBLISH_PERIOD_S:
            send()                            # an executor that just connected gets the plan at once

    node.create_subscription(String, WORLD_STATE_TOPIC, on_world_state,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_timer(0.5, on_timer)
    log(f"baseline planner up; waiting for {WORLD_STATE_TOPIC}")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    if len(sys.argv) == 2 and sys.argv[1] == "--selftest":
        sys.exit(0 if selftest() else 1)
    if len(sys.argv) == 1:
        run()
        sys.exit(0)
    sys.exit(__doc__)
