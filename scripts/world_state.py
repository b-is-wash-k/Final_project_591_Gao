#!/usr/bin/env python3
"""World state for the planners, in the format of docs/communication_standards.md.

Turns the TRUE world (config/scenario.yaml + live robot poses + switch/door
states from the switch manager) into what the TEAM KNOWS, as JSON:

    {"rooms": [...], "robots": [...], "switches": [...], "doors": [...],
     "tasks": [...], "costs": {...}}

Nothing that has not been discovered may appear in it (no "leaks").

Usage (from the repository root):
    python3 scripts/world_state.py --selftest   # check the functions, no Gazebo needed
    python3 scripts/world_state.py --demo       # print the JSON for two example situations

Decisions used here:
  - Discovery is per room: a room is known once a robot has been inside it.
  - A door is known if one of its rooms is known; the other side is "UNKNOWN" until entered.
  - A switch's "controls" and a door's "open_if" only mention things that are known.
"""
import json
import math
import sys

DEFAULT_STATUS = {"type": "ok", "repair_time": None, "waiting_for": None}


# ------------------------------------------- known view, tasks and path costs --

def room_of(xy, rooms):
    """Id of the room that contains point xy = (x, y), or None if it is in no room.

    rooms: list like [{"id": "1,1", "bounds": [x_min, y_min, x_max, y_max]}, ...]
    Bounds are inclusive. A point exactly on a shared wall belongs to the FIRST
    room in the list that contains it.
    Return the first room whose inclusive bounds contain xy.
    """
    x, y = xy

    for room in rooms:
        x_min, y_min, x_max, y_max = room["bounds"]
        if x_min <= x <= x_max and y_min <= y <= y_max:
            return room["id"]

    return None


def discover(known_rooms, robot_xy, rooms):
    """Discovery rule: a room becomes known as soon as a robot is inside it, and stays known.

    known_rooms: set of room ids known so far, e.g. {"1,1"}
    robot_xy:    {robot_id: (x, y)}
    Returns a NEW set (do not modify known_rooms).
    Return a new set containing previously known rooms and robots' rooms."""

    discovered = set(known_rooms)

    for xy in robot_xy.values():
        room_id = room_of(xy, rooms)
        if room_id is not None:
            discovered.add(room_id)

    return discovered


def known_view(sc, known_rooms, robot_xy, switch_on, door_open, robot_info):
    """The rooms / robots / switches / doors part of the world state: ONLY known things.

    sc:          the scenario dict (config/scenario.yaml)
    known_rooms: set of known room ids
    robot_xy:    {robot_id: (x, y)}
    switch_on:   {switch_id: True/False}   (from the switch manager)
    door_open:   {door_id: True/False}     (from the switch manager)
    robot_info:  {robot_id: {"status": {...}, "task": "HOLD:S1" or None}}; a missing
                 robot or key means status DEFAULT_STATUS and task None.

    What is known:
      - a switch or the exit: its pose is inside a known room (use room_of)
      - a door: at least one room in its `connects` is known

    Return {"rooms": [...], "robots": [...], "switches": [...], "doors": [...]}, each list
    sorted by id, with exactly these keys (see the expected JSON in selftest()):
      rooms:    {"id", "robots": [robot ids inside it now],
                 "switches": [known switch ids inside it],
                 "doors": [{"id", "other_room": the other connected room if known, else "UNKNOWN"}],
                 "exit": True if the exit is in this room}
      robots:   ALL robots: {"id", "colors", "status", "task"}
      switches: known switches: {"id", "color", "mode", "active": switch_on[id],
                 "controls": first known door (by id) whose `requires` lists this switch, else None}
      doors:    known doors: {"id", "open": door_open[id],
                 "open_if": {<door logic, e.g. "any_of">: [known switch ids from `requires`]}}
    """
    all_room_ids = {room["id"] for room in sc["rooms"]}
    known_room_ids = set(known_rooms) & all_room_ids

    # A switch or exit is known only when its position is in a known room.
    known_switches = [
        switch
        for switch in sc["switches"]
        if room_of((switch["pose"][0], switch["pose"][1]), sc["rooms"])
        in known_room_ids
    ]
    known_switches.sort(key=lambda switch: switch["id"])
    known_switch_ids = {switch["id"] for switch in known_switches}

    exit_xy = (sc["exit"]["pose"][0], sc["exit"]["pose"][1])
    exit_room = room_of(exit_xy, sc["rooms"])
    exit_is_known = exit_room in known_room_ids

    # A door is known as soon as at least one connected room is known.
    known_doors = [
        door
        for door in sc["doors"]
        if set(door["connects"]) & known_room_ids
    ]
    known_doors.sort(key=lambda door: door["id"])

    room_view = []
    known_rooms_sorted = sorted(
        (room for room in sc["rooms"] if room["id"] in known_room_ids),
        key=lambda room: room["id"],
    )

    for room in known_rooms_sorted:
        room_id = room["id"]

        robots_here = sorted(
            robot["id"]
            for robot in sc["robots"]
            if robot["id"] in robot_xy
            and room_of(robot_xy[robot["id"]], sc["rooms"]) == room_id
        )

        switches_here = sorted(
            switch["id"]
            for switch in known_switches
            if room_of(
                (switch["pose"][0], switch["pose"][1]), sc["rooms"]
            ) == room_id
        )

        doors_here = []
        for door in known_doors:
            if room_id not in door["connects"]:
                continue

            other_room = next(
                (
                    connected_room
                    for connected_room in door["connects"]
                    if connected_room != room_id
                    and connected_room in known_room_ids
                ),
                "UNKNOWN",
            )
            doors_here.append({
                "id": door["id"],
                "other_room": other_room,
            })

        room_view.append({
            "id": room_id,
            "robots": robots_here,
            "switches": switches_here,
            "doors": doors_here,
            "exit": exit_is_known and exit_room == room_id,
        })

    robot_view = []
    for robot in sorted(sc["robots"], key=lambda robot: robot["id"]):
        robot_id = robot["id"]
        info = robot_info.get(robot_id, {})

        robot_view.append({
            "id": robot_id,
            "colors": robot["colors"],
            "status": dict(info.get("status", DEFAULT_STATUS)),
            "task": info.get("task"),
        })

    switch_view = []
    for switch in known_switches:
        switch_id = switch["id"]

        # Sorting known_doors makes this the first known matching door by ID.
        controls = next(
            (
                door["id"]
                for door in known_doors
                if switch_id in door["requires"]
            ),
            None,
        )

        switch_view.append({
            "id": switch_id,
            "color": switch["color"],
            "mode": switch["mode"],
            "active": bool(switch_on.get(switch_id, False)),
            "controls": controls,
        })

    door_view = []
    for door in known_doors:
        known_required_switches = sorted(
            switch_id
            for switch_id in door["requires"]
            if switch_id in known_switch_ids
        )

        door_view.append({
            "id": door["id"],
            "open": bool(door_open.get(door["id"], False)),
            "open_if": {
                door["logic"]: known_required_switches,
            },
        })

    return {
        "rooms": room_view,
        "robots": robot_view,
        "switches": switch_view,
        "doors": door_view,
    }


def make_tasks(view):
    """The task list (communication_standards.md, "tasks") built from the known view.

    For each known switch, by switch id:
      - latch switch that is not active -> ACTIVATE:<S>
      - hold switch                     -> HOLD:<S>
      both with "needs_color": switch color, "enables": "CROSS:<controls>" (None if
      controls is None), "depends": None
    For each known door, by door id:
      - CROSS:<D> with "needs_color": None, "enables": None,
        "depends": sorted ["HOLD:<S>", ...] for the door's known HOLD switches, or None if none
    If the exit is known:
      - EXIT with "target": id of the room that has the exit; needs_color/enables/depends None

    Each task: {"id", "type", "target", "needs_color", "enables", "depends"}, where
    "target" is the switch id, door id, or room id. Order: switch tasks, CROSS tasks, EXIT.
    """
    tasks = []

    # Switch tasks come first, sorted by switch ID.
    for switch in sorted(view["switches"], key=lambda switch: switch["id"]):
        if switch["mode"] == "latch":
            # An active latch has already been activated, so it needs no task.
            if switch["active"]:
                continue
            task_type = "ACTIVATE"
        elif switch["mode"] == "hold":
            # Holding is an ongoing job, even if the switch is active now.
            task_type = "HOLD"
        else:
            raise ValueError(f"unknown switch mode: {switch['mode']}")

        switch_id = switch["id"]
        controls = switch["controls"]

        tasks.append({
            "id": f"{task_type}:{switch_id}",
            "type": task_type,
            "target": switch_id,
            "needs_color": switch["color"],
            "enables": f"CROSS:{controls}" if controls is not None else None,
            "depends": None,
        })

    # Then create one CROSS task for each known door.
    for door in sorted(view["doors"], key=lambda door: door["id"]):
        required_ids = {
            switch_id
            for switch_ids in door["open_if"].values()
            for switch_id in switch_ids
        }

        hold_switch_ids = sorted(
            switch["id"]
            for switch in view["switches"]
            if switch["mode"] == "hold"
            and switch["id"] in required_ids
        )
        depends = [f"HOLD:{switch_id}" for switch_id in hold_switch_ids]

        tasks.append({
            "id": f"CROSS:{door['id']}",
            "type": "CROSS",
            "target": door["id"],
            "needs_color": None,
            "enables": None,
            "depends": depends or None,
        })

    # Add EXIT last, if a known room contains the exit.
    exit_room = next(
        (room["id"] for room in view["rooms"] if room["exit"]),
        None,
    )
    if exit_room is not None:
        tasks.append({
            "id": "EXIT",
            "type": "EXIT",
            "target": exit_room,
            "needs_color": None,
            "enables": None,
            "depends": None,
        })

    return tasks


def path_cost(start, goal, sc, view):
    """Shortest path length in meters from start to goal on the KNOWN map, or None if unreachable.

    start, goal: {"xy": (x, y), "rooms": set of room ids the point is in, "door": door id or None}
                 (a door center is in BOTH rooms it connects; built for you by goal_of())
    sc:          the scenario (door poses and `connects`)
    view:        known_view(...) output: which doors are known and open, which rooms are known

    Rules:
      - Rooms are rectangles, so two points in the SAME room are joined by a straight line.
      - To change rooms, the path goes through door centers.
      - Only KNOWN doors can be used, and only if they are OPEN, except the goal door itself
        (CROSS:<D> only needs the robot to REACH the door, so a closed goal door is fine).
      - A door only joins its rooms that are known.
    Hint: nodes = start, goal and the usable door centers; connect two nodes if they share a
    room; run Dijkstra (there are only a few doors, so a simple version is fine).
    """
    known_room_ids = {room["id"] for room in view["rooms"]}
    known_door_by_id = {door["id"]: door for door in view["doors"]}

    # Ignore any room IDs that are not in the current known map.
    nodes = [
        {
            "xy": start["xy"],
            "rooms": set(start["rooms"]) & known_room_ids,
        },
        {
            "xy": goal["xy"],
            "rooms": set(goal["rooms"]) & known_room_ids,
        },
    ]

    goal_door_id = goal.get("door")

    # Add open, known doors as possible routes between rooms.
    # The goal door is already represented by the goal node; it may be closed
    # because CROSS only requires reaching it.
    for door in sorted(sc["doors"], key=lambda door: door["id"]):
        door_id = door["id"]
        visible_door = known_door_by_id.get(door_id)

        if visible_door is None:
            continue
        if door_id == goal_door_id:
            continue
        if not visible_door["open"]:
            continue

        connected_known_rooms = set(door["connects"]) & known_room_ids
        if not connected_known_rooms:
            continue

        nodes.append({
            "xy": (door["pose"][0], door["pose"][1]),
            "rooms": connected_known_rooms,
        })

    # Dijkstra's algorithm. Two nodes have a direct straight-line edge when
    # they share a known room.
    distances = [math.inf] * len(nodes)
    visited = [False] * len(nodes)
    distances[0] = 0.0

    while True:
        current = None
        for index in range(len(nodes)):
            if not visited[index] and (
                current is None or distances[index] < distances[current]
            ):
                current = index

        if current is None or math.isinf(distances[current]):
            return None

        if current == 1:
            return distances[current]

        visited[current] = True
        x1, y1 = nodes[current]["xy"]

        for neighbor in range(len(nodes)):
            if visited[neighbor]:
                continue
            if not (nodes[current]["rooms"] & nodes[neighbor]["rooms"]):
                continue

            x2, y2 = nodes[neighbor]["xy"]
            segment_length = math.hypot(x2 - x1, y2 - y1)
            candidate = distances[current] + segment_length

            if candidate < distances[neighbor]:
                distances[neighbor] = candidate


# ------------------------------------------------------------------ plumbing --

def goal_of(task, sc, view):
    """Where a task's target is, in the form path_cost() expects."""
    rooms = sc["rooms"]
    if task["type"] in ("ACTIVATE", "HOLD"):
        sw = next(s for s in sc["switches"] if s["id"] == task["target"])
        xy = (sw["pose"][0], sw["pose"][1])
        return {"xy": xy, "rooms": {room_of(xy, rooms)}, "door": None}
    if task["type"] == "CROSS":
        d = next(d for d in sc["doors"] if d["id"] == task["target"])
        known = {r["id"] for r in view["rooms"]}
        return {"xy": (d["pose"][0], d["pose"][1]), "rooms": set(d["connects"]) & known, "door": d["id"]}
    if task["type"] == "EXIT":
        xy = (sc["exit"]["pose"][0], sc["exit"]["pose"][1])
        return {"xy": xy, "rooms": {room_of(xy, rooms)}, "door": None}
    raise ValueError(f"no target position for task type {task['type']}")


def compute_costs(view, tasks, robot_xy, sc):
    """{robot_id: {task_id: path length rounded to 0.01 m}}; unreachable tasks are left out."""
    costs = {}
    for rid in sorted(robot_xy):
        xy = robot_xy[rid]
        start = {"xy": xy, "rooms": {room_of(xy, sc["rooms"])}, "door": None}
        costs[rid] = {}
        for t in tasks:
            c = path_cost(start, goal_of(t, sc, view), sc, view)
            if c is not None:
                costs[rid][t["id"]] = round(c, 2)
    return costs


def build_world_state(sc, known_rooms, robot_xy, switch_on, door_open, robot_info=None):
    """The full world-state dict for the planners."""
    view = known_view(sc, known_rooms, robot_xy, switch_on, door_open, robot_info or {})
    tasks = make_tasks(view)
    return {**view, "tasks": tasks, "costs": compute_costs(view, tasks, robot_xy, sc)}


# ------------------------------------------------------- self-test (no Gazebo) --

# A fixed copy of the two-room scenario, so the test doesn't change if config/scenario.yaml does.
TEST_SCENARIO = {
    "rooms": [{"id": "1,1", "bounds": [-6.0, -2.5, 0.0, 2.5]},
              {"id": "1,2", "bounds": [0.0, -2.5, 6.0, 2.5]}],
    "robots": [{"id": "robot1", "colors": ["red"]}, {"id": "robot2", "colors": ["blue"]}],
    "switches": [{"id": "S1", "color": "red", "mode": "hold", "pose": [-1.1, -1.45, 0.0]},
                 {"id": "S2", "color": "blue", "mode": "hold", "pose": [1.1, 1.45, 0.0]}],
    "doors": [{"id": "D1", "logic": "any_of", "requires": ["S1", "S2"],
               "pose": [0.0, 0.0, 0.0], "connects": ["1,1", "1,2"]}],
    "exit": {"id": "EXIT", "pose": [5.1, 0.0, 0.0]},
}
OK_STATUS = dict(DEFAULT_STATUS)
ROBOTS_ALL = [{"id": "robot1", "colors": ["red"], "status": OK_STATUS, "task": None},
              {"id": "robot2", "colors": ["blue"], "status": OK_STATUS, "task": None}]
TASKS_BOTH_ROOMS = [
    {"id": "HOLD:S1", "type": "HOLD", "target": "S1", "needs_color": "red", "enables": "CROSS:D1", "depends": None},
    {"id": "HOLD:S2", "type": "HOLD", "target": "S2", "needs_color": "blue", "enables": "CROSS:D1", "depends": None},
    {"id": "CROSS:D1", "type": "CROSS", "target": "D1", "needs_color": None, "enables": None,
     "depends": ["HOLD:S1", "HOLD:S2"]},
    {"id": "EXIT", "type": "EXIT", "target": "1,2", "needs_color": None, "enables": None, "depends": None},
]

# Situation A, t = 0: both robots at their starts, only room 1,1 known. Nothing about 1,2 may appear.
SITUATION_A = dict(
    known_rooms={"1,1"},
    robot_xy={"robot1": (-4.8, -0.65), "robot2": (-4.8, 0.65)},
    switch_on={"S1": False, "S2": False}, door_open={"D1": False})
EXPECTED_A = {
    "rooms": [{"id": "1,1", "robots": ["robot1", "robot2"], "switches": ["S1"],
               "doors": [{"id": "D1", "other_room": "UNKNOWN"}], "exit": False}],
    "robots": ROBOTS_ALL,
    "switches": [{"id": "S1", "color": "red", "mode": "hold", "active": False, "controls": "D1"}],
    "doors": [{"id": "D1", "open": False, "open_if": {"any_of": ["S1"]}}],
    "tasks": [
        {"id": "HOLD:S1", "type": "HOLD", "target": "S1", "needs_color": "red", "enables": "CROSS:D1", "depends": None},
        {"id": "CROSS:D1", "type": "CROSS", "target": "D1", "needs_color": None, "enables": None, "depends": ["HOLD:S1"]},
    ],
    "costs": {"robot1": {"HOLD:S1": 3.79, "CROSS:D1": 4.84},
              "robot2": {"HOLD:S1": 4.25, "CROSS:D1": 4.84}},
}

# Situation B, handoff: robot1 holds S1 (room 1,1), robot2 crossed and holds S2 (room 1,2), door open.
SITUATION_B = dict(
    known_rooms={"1,1", "1,2"},
    robot_xy={"robot1": (-1.1, -1.45), "robot2": (1.1, 1.45)},
    switch_on={"S1": True, "S2": True}, door_open={"D1": True})
EXPECTED_B = {
    "rooms": [{"id": "1,1", "robots": ["robot1"], "switches": ["S1"],
               "doors": [{"id": "D1", "other_room": "1,2"}], "exit": False},
              {"id": "1,2", "robots": ["robot2"], "switches": ["S2"],
               "doors": [{"id": "D1", "other_room": "1,1"}], "exit": True}],
    "robots": ROBOTS_ALL,
    "switches": [{"id": "S1", "color": "red", "mode": "hold", "active": True, "controls": "D1"},
                 {"id": "S2", "color": "blue", "mode": "hold", "active": True, "controls": "D1"}],
    "doors": [{"id": "D1", "open": True, "open_if": {"any_of": ["S1", "S2"]}}],
    "tasks": TASKS_BOTH_ROOMS,
    "costs": {"robot1": {"HOLD:S1": 0.0, "HOLD:S2": 3.64, "CROSS:D1": 1.82, "EXIT": 6.92},
              "robot2": {"HOLD:S1": 3.64, "HOLD:S2": 0.0, "CROSS:D1": 1.82, "EXIT": 4.25}},
}

# Situation C, door closed again: both rooms known, robot1 in 1,1, robot2 in 1,2, nobody on a switch.
# Anything on the other side of the closed door must be MISSING from that robot's costs.
SITUATION_C = dict(
    known_rooms={"1,1", "1,2"},
    robot_xy={"robot1": (-3.0, -1.45), "robot2": (4.0, 1.0)},
    switch_on={"S1": False, "S2": False}, door_open={"D1": False})
EXPECTED_C = {
    "rooms": [{"id": "1,1", "robots": ["robot1"], "switches": ["S1"],
               "doors": [{"id": "D1", "other_room": "1,2"}], "exit": False},
              {"id": "1,2", "robots": ["robot2"], "switches": ["S2"],
               "doors": [{"id": "D1", "other_room": "1,1"}], "exit": True}],
    "robots": ROBOTS_ALL,
    "switches": [{"id": "S1", "color": "red", "mode": "hold", "active": False, "controls": "D1"},
                 {"id": "S2", "color": "blue", "mode": "hold", "active": False, "controls": "D1"}],
    "doors": [{"id": "D1", "open": False, "open_if": {"any_of": ["S1", "S2"]}}],
    "tasks": TASKS_BOTH_ROOMS,
    "costs": {"robot1": {"HOLD:S1": 1.9, "CROSS:D1": 3.33},
              "robot2": {"HOLD:S2": 2.93, "CROSS:D1": 4.12, "EXIT": 1.49}},
}


def first_difference(got, want, path="world_state"):
    """Where two JSON-like values first differ, as a readable string (None if equal)."""
    if isinstance(want, dict) and isinstance(got, dict):
        for k in want:
            if k not in got:
                return f"{path}: missing key {k!r}"
            d = first_difference(got[k], want[k], f"{path}[{k!r}]")
            if d:
                return d
        extra = [k for k in got if k not in want]
        return f"{path}: unexpected key(s) {extra}" if extra else None
    if isinstance(want, list) and isinstance(got, list):
        if len(got) != len(want):
            return f"{path}: got {len(got)} items, want {len(want)}\n    got:  {got}\n    want: {want}"
        for i, (g, w) in enumerate(zip(got, want)):
            d = first_difference(g, w, f"{path}[{i}]")
            if d:
                return d
        return None
    if isinstance(want, float) or isinstance(got, float):
        if isinstance(got, (int, float)) and isinstance(want, (int, float)) and abs(got - want) < 0.005:
            return None
    if got != want or type(got) is not type(want):
        return f"{path}: got {got!r}, want {want!r}"
    return None


def selftest():
    sc = TEST_SCENARIO
    rooms = sc["rooms"]
    bad = 0

    def check(name, func):
        nonlocal bad
        try:
            problem = func()
        except NotImplementedError as e:
            problem = f"not written yet ({e})"
        except Exception as e:  # show the error instead of stopping the whole test
            problem = f"{type(e).__name__}: {e}"
        bad += bool(problem)
        print(f"  {'PASS' if not problem else 'FAIL'}  {name}" + (f"\n        {problem}" if problem else ""))

    def expect(got, want):
        return None if got == want else f"got {got!r}, want {want!r}"

    print("room_of:")
    check("start of robot1 is in 1,1", lambda: expect(room_of((-4.8, -0.65), rooms), "1,1"))
    check("exit is in 1,2", lambda: expect(room_of((5.1, 0.0), rooms), "1,2"))
    check("door center (on the shared wall) -> first room, 1,1", lambda: expect(room_of((0.0, 0.0), rooms), "1,1"))
    check("outside the maze -> None", lambda: expect(room_of((7.0, 0.0), rooms), None))

    print("discover:")
    check("robots in 1,1 -> {1,1}", lambda: expect(
        discover(set(), {"robot1": (-4.8, -0.65), "robot2": (-4.8, 0.65)}, rooms), {"1,1"}))
    check("robot2 enters 1,2 -> {1,1, 1,2}", lambda: expect(
        discover({"1,1"}, {"robot1": (-1.1, -1.45), "robot2": (1.1, 1.45)}, rooms), {"1,1", "1,2"}))
    check("rooms stay known after robots leave", lambda: expect(
        discover({"1,1", "1,2"}, {"robot1": (-4.8, -0.65), "robot2": (-4.8, 0.65)}, rooms), {"1,1", "1,2"}))

    def does_not_modify_input():
        known = {"1,1"}
        discover(known, {"robot2": (1.1, 1.45)}, rooms)
        return None if known == {"1,1"} else f"known_rooms was changed to {known}"
    check("returns a new set (input unchanged)", does_not_modify_input)

    print("full world state (known_view + make_tasks + path_cost):")
    for name, situation, expected in [
            ("A: t = 0, only room 1,1 known", SITUATION_A, EXPECTED_A),
            ("B: handoff, door open", SITUATION_B, EXPECTED_B),
            ("C: door closed, unreachable targets left out", SITUATION_C, EXPECTED_C)]:
        check(name, lambda s=situation, e=expected: first_difference(build_world_state(sc, **s), e))

    def no_leak_at_t0():
        text = json.dumps(build_world_state(sc, **SITUATION_A))
        leaks = [word for word in ("S2", "1,2", "EXIT") if word in text]
        return f"hidden things appear at t = 0: {leaks}" if leaks else None
    check("A: no leak (no S2, no room 1,2, no EXIT at t = 0)", no_leak_at_t0)

    print("ALL PASS" if bad == 0 else f"{bad} FAILED")
    return bad == 0


def demo():
    for name, s in [("Situation A (t = 0)", SITUATION_A), ("Situation B (handoff)", SITUATION_B)]:
        print(f"===== {name} =====")
        print(json.dumps(build_world_state(TEST_SCENARIO, **s), indent=2))


if __name__ == "__main__":
    arg = sys.argv[1] if len(sys.argv) == 2 else ""
    if arg == "--selftest":
        sys.exit(0 if selftest() else 1)
    if arg == "--demo":
        demo()
        sys.exit(0)
    sys.exit(__doc__)
