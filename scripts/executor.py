#!/usr/bin/env python3
"""Plan executor: turns the planner's task for each robot into Nav2 goals.

    /team/plan (from ANY planner)  ->  executor  ->  /<robot>/navigate_to_pose (Nav2)
    /team/world_state              ->  executor  ->  /team/robot_status -> switch manager -> world state

Input: the planner output of docs/communication_standards.md, a std_msgs/String with JSON
like {"robot1": "HOLD:S1", "robot2": "CROSS:D1"}. Every planner (the baseline planner, the
SMART-LLM planner, a hand-written plan from test_helpers.sh) uses this same topic and format.

What each task means for a robot:
  ACTIVATE:<S>  drive onto the switch pad; done once the switch is active.
  HOLD:<S>      drive onto the pad and stay there; never "done" (the planner moves it on).
  CROSS:<D>     drive through the door into the room on the other side; done once past it.
                If the door is closed, wait in front of it (status "waiting") until it opens.
                This also works for a door whose other side is still UNKNOWN (exploring).
  EXIT          drive to a free spot next to the exit; done when there.
  WAIT          stop and do nothing.
A task id that is re-sent keeps the current goal. A finished task that is re-sent is ignored.
When a task is done, the robot's "task" goes back to null in /team/robot_status (and so in the
world state), which tells the planners to replan.

The executor drives the real robot, so it may use the true geometry in config/scenario.yaml
(where a door is and which rooms it joins). The planners only ever see the world state.

Usage (from the repository root; use the system python3):
    python3 scripts/executor.py --selftest            # check the task logic, no ROS or Gazebo needed
    python3 scripts/executor.py config/scenario.yaml  # run (Gazebo, robots and Nav2 must be up)
"""
import json
import math
import sys
import time

import yaml

from world_state import room_of

ROBOT_STATUS_TOPIC = "/team/robot_status"
PLAN_TOPIC = "/team/plan"
WORLD_STATE_TOPIC = "/team/world_state"
TASK_TYPES = ("ACTIVATE", "HOLD", "CROSS", "EXIT")

CROSS_BEYOND_M = 0.9      # CROSS goal: this far past the door center, inside the far room
CROSS_STAGING_M = 0.9     # waiting spot: this far in front of a closed door
CROSS_DONE_M = 0.6        # CROSS is done once the robot center is this far past the door center
CROSS_COMMITTED_M = 0.2   # closer than this to the door plane: keep going even if the door closes
DOORWAY_HALF_WIDTH_M = 1.0  # a robot this close to a door center, along the wall, is "in the doorway"
WALL_MARGIN_M = 0.4       # goals stay at least this far from a room's walls
EXIT_SLOT_RADIUS_M = 0.6  # robots park on a circle of this radius around the exit
EXIT_SLOT_SPACING_M = 0.6 # minimum distance between two parked robots
REACHED_M = 0.25          # "at the goal" when this close (Nav2's goal tolerance is 0.12 m)
ACTIVATE_GIVE_UP_S = 5.0  # ACTIVATE ends this long after reaching the pad even if the switch stays off
STEP_HZ = 5
STATUS_PERIOD_S = 1.0     # /team/robot_status is re-sent at least this often


# --------------------------------------------------------- task logic (no ROS) --

def parse_task(task_id):
    """"HOLD:S1" -> ("HOLD", "S1"); "EXIT" -> ("EXIT", None); anything else -> (None, None)."""
    if not isinstance(task_id, str):
        return None, None
    kind, _, target = task_id.partition(":")
    if kind == "EXIT" and not target:
        return "EXIT", None
    if kind in ("ACTIVATE", "HOLD", "CROSS") and target:
        return kind, target
    return None, None


def check_task(task_id, sc):
    """None if the task id can be executed in this scenario, else the reason it can't."""
    kind, target = parse_task(task_id)
    if kind is None:
        return f"unknown task id {task_id!r}"
    if kind in ("ACTIVATE", "HOLD") and not any(s["id"] == target for s in sc["switches"]):
        return f"no switch {target!r} in the scenario"
    if kind == "CROSS" and not any(d["id"] == target for d in sc["doors"]):
        return f"no door {target!r} in the scenario"
    return None


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def room_by_id(sc, room_id):
    return next(r for r in sc["rooms"] if r["id"] == room_id)


def clamp_into(room, xy, margin=WALL_MARGIN_M):
    """xy moved inside the room's bounds, at least `margin` from every wall."""
    x0, y0, x1, y1 = room["bounds"]
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    lo_x, hi_x = (x0 + margin, x1 - margin) if x1 - x0 > 2 * margin else (cx, cx)
    lo_y, hi_y = (y0 + margin, y1 - margin) if y1 - y0 > 2 * margin else (cy, cy)
    return (min(max(xy[0], lo_x), hi_x), min(max(xy[1], lo_y), hi_y))


def into_room(door_xy, room):
    """Unit vector from a door center into the room, perpendicular to the wall the door is in.

    The door is in the room wall nearest to its center.
    """
    x, y = door_xy
    x0, y0, x1, y1 = room["bounds"]
    walls = [(abs(x - x0), (1.0, 0.0)), (abs(x - x1), (-1.0, 0.0)),
             (abs(y - y0), (0.0, 1.0)), (abs(y - y1), (0.0, -1.0))]
    return min(walls)[1]


def cross_plan(door, robot_xy, sc):
    """How to cross `door` from where the robot is now.

    Returns {"near", "far": room ids, "normal": unit vector into the far room,
             "staging": waiting spot in front of the door, "beyond": goal in the far room}.
    The far room is the connected room the robot is NOT in. A robot in neither room counts
    as being on the side of the room whose center is closer.
    """
    a, b = door["connects"]
    here = room_of(robot_xy, sc["rooms"])
    if here == b:
        near, far = b, a
    elif here == a:
        near, far = a, b
    else:
        center = lambda rid: ((room_by_id(sc, rid)["bounds"][0] + room_by_id(sc, rid)["bounds"][2]) / 2,
                              (room_by_id(sc, rid)["bounds"][1] + room_by_id(sc, rid)["bounds"][3]) / 2)
        near, far = (a, b) if dist(robot_xy, center(a)) <= dist(robot_xy, center(b)) else (b, a)
    door_xy = (door["pose"][0], door["pose"][1])
    n = into_room(door_xy, room_by_id(sc, far))
    staging = clamp_into(room_by_id(sc, near),
                         (door_xy[0] - n[0] * CROSS_STAGING_M, door_xy[1] - n[1] * CROSS_STAGING_M))
    beyond = clamp_into(room_by_id(sc, far),
                        (door_xy[0] + n[0] * CROSS_BEYOND_M, door_xy[1] + n[1] * CROSS_BEYOND_M))
    return {"near": near, "far": far, "normal": n, "staging": staging, "beyond": beyond}


def progress_past(door, plan, robot_xy):
    """Signed distance (m) of the robot center past the door center, toward the far room."""
    n = plan["normal"]
    return (robot_xy[0] - door["pose"][0]) * n[0] + (robot_xy[1] - door["pose"][1]) * n[1]


def cross_action(door_open, progress):
    """What a crossing robot does next: "done", "beyond" (drive through) or "staging" (wait)."""
    if progress >= CROSS_DONE_M:
        return "done"
    if door_open or progress > -CROSS_COMMITTED_M:
        return "beyond"          # open, or already in the doorway: keep going
    return "staging"


def exit_slot(exit_xy, robot_xy, taken, room):
    """A parking spot next to the exit for a robot arriving from robot_xy.

    Spots lie on a circle around the exit, starting with the side the robot comes from, and
    keep EXIT_SLOT_SPACING_M from spots other robots have already taken.
    """
    base = math.atan2(robot_xy[1] - exit_xy[1], robot_xy[0] - exit_xy[0])
    for k in range(0, 13):
        turn = math.radians(30 * ((k + 1) // 2)) * (1 if k % 2 else -1)
        a = base + turn
        spot = clamp_into(room, (exit_xy[0] + EXIT_SLOT_RADIUS_M * math.cos(a),
                                 exit_xy[1] + EXIT_SLOT_RADIUS_M * math.sin(a)), margin=0.35)
        if all(dist(spot, t) >= EXIT_SLOT_SPACING_M for t in taken):
            return spot
    return clamp_into(room, exit_xy, margin=0.35)


def in_doorway(door, xy, sc):
    """True if a robot at xy is in the door's opening, where a closing gate would hit it."""
    n = into_room((door["pose"][0], door["pose"][1]), room_by_id(sc, door["connects"][0]))
    dx, dy = xy[0] - door["pose"][0], xy[1] - door["pose"][1]
    across = abs(dx * n[0] + dy * n[1])          # distance from the wall line
    along = abs(dx * n[1] - dy * n[0])           # distance along the wall from the door center
    return across < CROSS_DONE_M and along < DOORWAY_HALF_WIDTH_M


def holder_must_stay(switch_id, robot_id, sc, ws, robot_xy):
    """(door id, robot id) if the robot holding switch_id must stay on it, else None.

    A holder stays while another robot is in the opening of a door that would close without
    this switch. This protects a crossing robot whatever the planner says.
    """
    for door in sc["doors"]:
        if switch_id not in door["requires"]:
            continue
        others_on = [s for s in door["requires"] if s != switch_id and switch_active(ws, s)]
        if door["logic"] == "any_of" and others_on:
            continue                                # another switch keeps it open
        for rid, xy in robot_xy.items():
            if rid != robot_id and xy is not None and in_doorway(door, xy, sc):
                return door["id"], rid
    return None


def waiting_for(door_id, ws):
    """The tasks a robot waiting at a closed door is waiting for (from the world state)."""
    tasks = {t["id"]: t for t in (ws or {}).get("tasks", [])}
    cross = tasks.get(f"CROSS:{door_id}")
    if cross and cross.get("depends"):
        return list(cross["depends"])
    enablers = sorted(t["id"] for t in tasks.values() if t.get("enables") == f"CROSS:{door_id}")
    return enablers or None


def switch_active(ws, switch_id):
    return any(s["id"] == switch_id and s.get("active") for s in (ws or {}).get("switches", []))


def door_open(ws, door_id):
    return any(d["id"] == door_id and d.get("open") for d in (ws or {}).get("doors", []))


# ------------------------------------------------------- self-test (no ROS) --

def selftest():
    from world_state import TEST_SCENARIO as sc
    door = sc["doors"][0]
    bad = 0

    def check(name, got, want):
        nonlocal bad
        ok = got == want
        numbers = lambda t: isinstance(t, tuple) and all(isinstance(v, (int, float)) for v in t)
        if numbers(got) and numbers(want) and len(got) == len(want):
            ok = all(abs(g - w) < 1e-6 for g, w in zip(got, want))   # points: allow rounding
        bad += not ok
        print(f"  {'PASS' if ok else 'FAIL'}  {name}" + ("" if ok else f"\n        got {got!r}, want {want!r}"))

    print("task ids:")
    check("HOLD:S1", parse_task("HOLD:S1"), ("HOLD", "S1"))
    check("EXIT", parse_task("EXIT"), ("EXIT", None))
    check("EXPLORE:1,2 is not a task", parse_task("EXPLORE:1,2"), (None, None))
    check("CROSS:D9 is not in the scenario", check_task("CROSS:D9", sc), "no door 'D9' in the scenario")
    check("CROSS:D1 is fine", check_task("CROSS:D1", sc), None)

    print("crossing D1 (wall at x = 0 between 1,1 and 1,2):")
    west = cross_plan(door, (-4.8, 0.65), sc)
    check("from 1,1: far room is 1,2", (west["near"], west["far"]), ("1,1", "1,2"))
    check("from 1,1: goal beyond the door", west["beyond"], (0.9, 0.0))
    check("from 1,1: waiting spot in front of the door", west["staging"], (-0.9, 0.0))
    east = cross_plan(door, (4.0, 1.0), sc)
    check("from 1,2: far room is 1,1 (crossing back)", (east["near"], east["far"]), ("1,2", "1,1"))
    check("from 1,2: goal beyond the door", east["beyond"], (-0.9, 0.0))
    check("progress of a robot 0.7 m into 1,2", round(progress_past(door, west, (0.7, 0.3)), 6), 0.7)

    print("crossing rules:")
    check("door closed, robot far away -> wait in front", cross_action(False, -3.0), "staging")
    check("door open -> drive through", cross_action(True, -3.0), "beyond")
    check("door closes while robot is in the doorway -> keep going", cross_action(False, -0.1), "beyond")
    check("0.6 m past the door -> done", cross_action(False, 0.6), "done")
    ws = {"tasks": [{"id": "CROSS:D1", "depends": ["HOLD:S1"], "enables": None},
                    {"id": "HOLD:S1", "depends": None, "enables": "CROSS:D1"}]}
    check("waiting for the door's HOLD tasks", waiting_for("D1", ws), ["HOLD:S1"])
    ws_latch = {"tasks": [{"id": "CROSS:D1", "depends": None, "enables": None},
                          {"id": "ACTIVATE:S4", "depends": None, "enables": "CROSS:D1"}]}
    check("latch door: waiting for ACTIVATE", waiting_for("D1", ws_latch), ["ACTIVATE:S4"])

    print("holder interlock (D1 any_of S1, S2):")
    s2_only = {"switches": [{"id": "S1", "active": False}, {"id": "S2", "active": True}]}
    both = {"switches": [{"id": "S1", "active": True}, {"id": "S2", "active": True}]}
    check("robot1 in the doorway, only S2 on: robot2 stays on S2",
          holder_must_stay("S2", "robot2", sc, s2_only, {"robot1": (0.3, 0.0), "robot2": (1.1, 1.45)}),
          ("D1", "robot1"))
    check("robot1 clear of the doorway: robot2 may leave",
          holder_must_stay("S2", "robot2", sc, s2_only, {"robot1": (0.9, 0.0), "robot2": (1.1, 1.45)}), None)
    check("S1 also on: robot2 may leave",
          holder_must_stay("S2", "robot2", sc, both, {"robot1": (0.3, 0.0), "robot2": (1.1, 1.45)}), None)

    print("exit spots:")
    exit_xy, room = (5.1, 0.0), sc["rooms"][1]
    first = exit_slot(exit_xy, (0.9, 0.0), [], room)
    check("first robot parks on its own side of the exit", first, (4.5, 0.0))
    second = exit_slot(exit_xy, (1.1, 1.45), [first], room)
    check("second robot keeps its distance", dist(second, first) >= EXIT_SLOT_SPACING_M, True)
    check("second robot stays inside the room", second == clamp_into(room, second, margin=0.35), True)

    print("ALL PASS" if bad == 0 else f"{bad} FAILED")
    return bad == 0


# --------------------------------------------------------------- ROS 2 node --

class RobotRun:
    """What one robot is doing."""

    def __init__(self, rid, start_pose):
        self.rid = rid
        x, y, yaw = (list(start_pose) + [0.0, 0.0, 0.0])[:3]
        self.start = (x, y, yaw)            # map -> odom (see launch/navigation.launch.py)
        self.xy = None                      # position in the map frame, from odometry
        self.task = None                    # current task id, or None
        self.done_task = None               # last finished task (a re-sent copy is ignored)
        self.pending = None                 # next task, deferred while the robot must keep holding
        self.status = "ok"                  # "ok" or "waiting"
        self.waiting_for = None
        self.cross = None                   # cross_plan() for a CROSS task
        self.exit_spot = None
        self.arrived = None                 # when the robot reached its switch pad
        # Nav2 goal bookkeeping
        self.goal_xy = None
        self.nav = None                     # None | "pending" | "active" | "succeeded" | "failed"
        self.handle = None
        self.gen = 0                        # increases with every goal; stale callbacks are ignored
        self.failures = 0
        self.retry_at = 0.0


def run(scenario_path):
    import rclpy
    from action_msgs.msg import GoalStatus
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import Odometry
    from rclpy.action import ActionClient
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from std_msgs.msg import String

    with open(scenario_path) as f:
        sc = yaml.safe_load(f)
    radius = sc["scenario"].get("on_pad_radius_m", 0.30)
    switches = {s["id"]: s for s in sc["switches"]}
    doors = {d["id"]: d for d in sc["doors"]}
    exit_xy = (sc["exit"]["pose"][0], sc["exit"]["pose"][1])
    exit_room = room_by_id(sc, room_of(exit_xy, sc["rooms"]))

    rclpy.init()
    node = rclpy.create_node("executor")
    t0 = time.time()
    log = lambda text: print(f"[{time.time() - t0:7.2f}s] {text}", flush=True)
    robots = {r["id"]: RobotRun(r["id"], r["start_pose"]) for r in sc["robots"]}
    clients = {rid: ActionClient(node, NavigateToPose, f"/{rid}/navigate_to_pose") for rid in robots}
    latest = {"ws": None}
    status_pub = node.create_publisher(
        String, ROBOT_STATUS_TOPIC, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    last_status = {"text": None, "time": 0.0}

    def publish_status(force=False):
        text = json.dumps({rid: {"task": r.task,
                                 "status": {"type": r.status, "repair_time": None,
                                            "waiting_for": r.waiting_for}}
                           for rid, r in robots.items()})
        now = time.time()
        if force or text != last_status["text"] or now - last_status["time"] >= STATUS_PERIOD_S:
            status_pub.publish(String(data=text))
            last_status.update(text=text, time=now)

    # ---- inputs
    def on_odom(rid, msg):
        r = robots[rid]
        x0, y0, yaw0 = r.start
        px, py = msg.pose.pose.position.x, msg.pose.pose.position.y
        r.xy = (x0 + math.cos(yaw0) * px - math.sin(yaw0) * py,
                y0 + math.sin(yaw0) * px + math.cos(yaw0) * py)

    for rid in robots:
        node.create_subscription(Odometry, f"/{rid}/odom", lambda m, rid=rid: on_odom(rid, m),
                                 qos_profile_sensor_data)

    def on_world_state(msg):
        try:
            latest["ws"] = json.loads(msg.data)
        except json.JSONDecodeError as e:
            log(f"ignoring a world state that is not JSON: {e}")

    node.create_subscription(String, WORLD_STATE_TOPIC, on_world_state,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def on_plan(msg):
        try:
            plan = json.loads(msg.data)
        except json.JSONDecodeError as e:
            log(f"ignoring a plan that is not JSON: {e}")
            return
        if not isinstance(plan, dict):
            log(f"ignoring a plan that is not a JSON object: {msg.data[:80]}")
            return
        for rid, task in plan.items():
            if rid not in robots:
                log(f"plan names unknown robot {rid!r}; ignored")
                continue
            assign(robots[rid], task)
        publish_status()

    node.create_subscription(String, PLAN_TOPIC, on_plan, 10)   # planners publish volatile

    # ---- tasks
    def must_stay(r):
        """(door, robot) if r holds a switch that a robot in a doorway depends on, else None."""
        kind, target = parse_task(r.task)
        if kind != "HOLD" or not switch_active(latest["ws"], target):
            return None                             # not holding a switch that is on
        return holder_must_stay(target, r.rid, sc, latest["ws"], {o.rid: o.xy for o in robots.values()})

    def assign(r, task):
        task = "WAIT" if task is None else task
        if task != r.task:
            blocker = must_stay(r)
            if blocker:                             # interlock: don't drop a gate onto a robot
                if r.pending != task:
                    log(f"{r.rid}: stays on {parse_task(r.task)[1]} until {blocker[1]} is through "
                        f"{blocker[0]}, then {task}")
                r.pending = task
                return
        r.pending = None
        if task == "WAIT":
            if r.task is not None:                  # an idle robot keeps its last finished task
                log(f"{r.rid}: WAIT (stops {r.task})")
                r.done_task = None
            cancel(r)
            r.task, r.status, r.waiting_for = None, "ok", None
            return
        if task == r.task:
            return                                  # same task again: keep going
        if r.task is None and task == r.done_task:
            return                                  # just finished this one
        problem = check_task(task, sc)
        if problem:
            log(f"{r.rid}: cannot execute {task!r} ({problem}); keeps {r.task}")
            return
        log(f"{r.rid}: new task {task}" + (f" (was {r.task})" if r.task else ""))
        r.task, r.done_task = task, None
        r.status, r.waiting_for, r.cross, r.exit_spot = "ok", None, None, None
        r.failures, r.retry_at, r.arrived = 0, 0.0, None
        r.goal_xy = None                            # the next step sends a fresh goal

    def finish(r, note=""):
        log(f"{r.rid}: {r.task} done{note}")
        r.done_task, r.task = r.task, None
        r.status, r.waiting_for = "ok", None
        publish_status(force=True)

    # ---- Nav2 goals
    def cancel(r):
        r.gen += 1
        if r.handle is not None:
            r.handle.cancel_goal_async()
        r.handle, r.nav, r.goal_xy = None, None, None

    def go_to(r, xy):
        """Make sure the robot is driving to xy (sends a goal only when needed)."""
        if r.goal_xy == xy and r.nav in ("pending", "active", "succeeded"):
            return
        if r.goal_xy == xy and r.nav == "failed" and time.time() < r.retry_at:
            return
        client = clients[r.rid]
        if not client.server_is_ready():
            if r.nav != "no_server":
                log(f"{r.rid}: waiting for Nav2 (/{r.rid}/navigate_to_pose)")
            r.nav, r.goal_xy = "no_server", None
            return
        if r.goal_xy != xy:
            r.failures = 0
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"           # stamp 0 = use the latest transform
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(xy[0]), float(xy[1])
        goal.pose.pose.orientation.w = 1.0
        r.gen += 1
        gen = r.gen
        r.goal_xy, r.nav, r.handle = xy, "pending", None
        client.send_goal_async(goal).add_done_callback(lambda f: on_goal_response(r, gen, f))

    def on_goal_response(r, gen, future):
        handle = future.result()
        if gen != r.gen:
            if handle.accepted:
                handle.cancel_goal_async()          # a newer goal or a WAIT replaced this one
            return
        if not handle.accepted:
            nav_failed(r, "goal rejected")
            return
        r.handle, r.nav = handle, "active"
        handle.get_result_async().add_done_callback(lambda f: on_result(r, gen, f))

    def on_result(r, gen, future):
        if gen != r.gen:
            return
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            r.nav, r.failures = "succeeded", 0
        else:
            nav_failed(r, {GoalStatus.STATUS_ABORTED: "aborted", GoalStatus.STATUS_CANCELED: "canceled"}
                       .get(status, f"status {status}"))

    def nav_failed(r, why):
        r.failures += 1
        delay = min(2.0 * r.failures, 10.0)
        r.nav, r.handle, r.retry_at = "failed", None, time.time() + delay
        log(f"{r.rid}: Nav2 goal {r.goal_xy} for {r.task} failed ({why}, {r.failures}x); "
            f"retrying in {delay:.0f} s")

    # ---- one control step per robot
    def step(r):
        if r.task is None or r.xy is None:
            return
        ws = latest["ws"]
        kind, target = parse_task(r.task)
        if kind in ("ACTIVATE", "HOLD"):
            pad = (switches[target]["pose"][0], switches[target]["pose"][1])
            if kind == "ACTIVATE" and switch_active(ws, target):
                finish(r)
                return
            on_pad = dist(r.xy, pad) <= radius
            if on_pad and r.nav == "succeeded":
                r.arrived = r.arrived or time.time()
                if kind == "ACTIVATE" and time.time() - r.arrived > ACTIVATE_GIVE_UP_S:
                    finish(r, " (on the pad, but the switch did not turn on: wrong color?)")
                return
            r.arrived = None
            if not on_pad and r.nav == "succeeded":
                r.nav = None                        # pushed off the pad: drive back
            go_to(r, pad)
        elif kind == "CROSS":
            door = doors[target]
            if r.cross is None:
                r.cross = cross_plan(door, r.xy, sc)
            action = cross_action(door_open(ws, target), progress_past(door, r.cross, r.xy))
            reached_far_goal = r.nav == "succeeded" and r.goal_xy == r.cross["beyond"]
            if action == "done" or reached_far_goal:
                finish(r)
                return
            if action == "beyond":
                if r.status != "ok":
                    log(f"{r.rid}: {target} is open, crossing")
                r.status, r.waiting_for = "ok", None
                go_to(r, r.cross["beyond"])
            else:
                go_to(r, r.cross["staging"])
                at_door = dist(r.xy, r.cross["staging"]) <= REACHED_M or r.nav == "succeeded"
                if at_door and r.status != "waiting" and r.goal_xy == r.cross["staging"]:
                    r.status, r.waiting_for = "waiting", waiting_for(target, ws)
                    log(f"{r.rid}: waiting at closed {target} for {r.waiting_for}")
        elif kind == "EXIT":
            if r.exit_spot is None:
                taken = [o.exit_spot for o in robots.values() if o is not r and o.exit_spot]
                r.exit_spot = exit_slot(exit_xy, r.xy, taken, exit_room)
            if dist(r.xy, r.exit_spot) <= REACHED_M or (r.nav == "succeeded" and r.goal_xy == r.exit_spot):
                finish(r)
                if all(o.done_task == "EXIT" and o.task is None for o in robots.values()):
                    log("all robots are at the exit")
                return
            go_to(r, r.exit_spot)

    def on_step():
        for r in robots.values():
            if r.pending is not None and must_stay(r) is None:
                assign(r, r.pending)                # the doorway is clear: do the deferred task
            step(r)
        publish_status()

    node.create_timer(1.0 / STEP_HZ, on_step)
    log(f"executor up; robots {list(robots)}; listening on {PLAN_TOPIC}")
    publish_status(force=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        for r in robots.values():
            cancel(r)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    if len(sys.argv) == 2 and sys.argv[1] == "--selftest":
        sys.exit(0 if selftest() else 1)
    if len(sys.argv) == 2:
        run(sys.argv[1])
        sys.exit(0)
    sys.exit(__doc__)
