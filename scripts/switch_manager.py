#!/usr/bin/env python3
"""Switch manager: robot on a pad -> switch state -> gate open/closed.

Reads the true robot positions from Gazebo (not odometry), decides which
switches are ON and which doors are OPEN, and sends each gate's target
position to the Gazebo topic /maze/door/<ID>/position.

    robot poses (Gazebo) -> switch manager -> gate controller -> gate moves

It also builds the planners' world state (scripts/world_state.py) and publishes
it as JSON on the ROS 2 topic /team/world_state. Each robot's "task" and "status"
in it come from the executor, on /team/robot_status.

Usage (from the repository root; use the system python3):
    python3 scripts/switch_manager.py --selftest            # check the rules, no Gazebo needed
    python3 scripts/switch_manager.py config/scenario.yaml  # run with Gazebo
"""
import json
import math
import sys
import time

import yaml

import world_state

RATE_HZ = 10
WORLD_STATE_TOPIC = "/team/world_state"
ROBOT_STATUS_TOPIC = "/team/robot_status"
STATUS_TYPES = ("ok", "broken", "waiting")
COSTS_ONLY_MIN_PERIOD_S = 0.5    # when only costs change (robots moving), publish at most this often
HEARTBEAT_PERIOD_S = 2.0         # re-send the latest state at least this often, even if nothing changed


# -------------------------------------------------------------- puzzle rules --

def robot_on_pad(robot_xy, switch, radius):
    """True if the robot's center is on this switch pad.

    robot_xy: (x, y) of the robot in meters, world frame.
    switch:   one entry of `switches` in the scenario, e.g.
              {"id": "S1", "color": "red", "mode": "hold", "pose": [-1.1, -1.45, 0.0]}
    radius:   on_pad_radius_m from the scenario.
    Rule: distance from robot center to pad center <= radius.
    """
    pad_x, pad_y = switch["pose"][0], switch["pose"][1]
    dx = robot_xy[0] - pad_x
    dy = robot_xy[1] - pad_y
    return math.hypot(dx, dy) <= radius


def switch_is_on(switch, was_on, colors_on_pad):
    """New ON/OFF state of one switch for this time step.

    was_on:        the switch's state in the previous step (True/False).
    colors_on_pad: set of colors of the robots standing on this pad right now,
                   e.g. set(), {"red"}, {"red", "blue"}.
    Rules:
      - The switch is only pressed by a robot of the SAME color.
      - hold:  ON only while pressed.
      - latch: ON once pressed, and stays ON forever after.
    """
    pressed = switch["color"] in colors_on_pad

    if switch["mode"] == "hold":
        return pressed
    if switch["mode"] == "latch":
        return was_on or pressed

    raise ValueError(f"Unknown switch mode: {switch['mode']}")


def door_should_open(door, switch_on):
    """True if this door should be open.

    door:      one entry of `doors`, e.g. {"id": "D1", "logic": "any_of", "requires": ["S1", "S2"], ...}
    switch_on: dict switch id -> True/False, e.g. {"S1": True, "S2": False}
    Rules: any_of = at least one required switch ON; all_of = every required switch ON.
    """
    required_states = [switch_on[sid] for sid in door["requires"]]

    if door["logic"] == "any_of":
        return any(required_states)
    if door["logic"] == "all_of":
        return all(required_states)

    raise ValueError(f"Unknown door logic: {door['logic']}")


def robot_info_from_status(text, robot_ids):
    """A /team/robot_status message (JSON text) -> world_state's robot_info, or None if invalid.

    Expected: {"robot1": {"task": "HOLD:S1" or null,
                          "status": {"type": "ok"|"broken"|"waiting", "repair_time", "waiting_for"}}}
    Unknown robots and malformed entries are skipped; missing or bad fields get the defaults.
    """
    try:
        data = json.loads(text)
    except (TypeError, ValueError):
        return None
    if not isinstance(data, dict):
        return None
    info = {}
    for rid, entry in data.items():
        if rid not in robot_ids or not isinstance(entry, dict):
            continue
        task = entry.get("task")
        status = entry.get("status") if isinstance(entry.get("status"), dict) else {}
        repair = status.get("repair_time")
        waiting = status.get("waiting_for")
        info[rid] = {
            "task": task if isinstance(task, str) else None,
            "status": {
                "type": status.get("type") if status.get("type") in STATUS_TYPES else "ok",
                "repair_time": repair if isinstance(repair, (int, float)) and not isinstance(repair, bool) else None,
                "waiting_for": [w for w in waiting if isinstance(w, str)] if isinstance(waiting, list) else None,
            },
        }
    return info


# ------------------------------------------------------- self-test (no Gazebo) --

def selftest():
    # old: red_hold = {"id": "S_R", "color": "red", "mode": "hold", "pose": [-1.1, -1.45, 0.0]}
    red_hold = {"id": "S1", "color": "red", "mode": "hold", "pose": [-1.1, -1.45, 0.0]}
    red_latch = {"id": "S_L", "color": "red", "mode": "latch", "pose": [0.0, 0.0, 0.0]}
    # old: any_door = {"id": "D1", "logic": "any_of", "requires": ["S_R", "S_B"]}
    any_door = {"id": "D1", "logic": "any_of", "requires": ["S1", "S2"]}
    # old: all_door = {"id": "D2", "logic": "all_of", "requires": ["S_R", "S_B"]}
    all_door = {"id": "D2", "logic": "all_of", "requires": ["S1", "S2"]}
    cases = [
        ("on pad center",              robot_on_pad((-1.1, -1.45), red_hold, 0.30), True),
        ("0.25 m away (inside)",       robot_on_pad((-0.85, -1.45), red_hold, 0.30), True),
        ("0.45 m away",                robot_on_pad((-1.1, -1.00), red_hold, 0.30), False),
        ("hold: red robot on red pad", switch_is_on(red_hold, False, {"red"}), True),
        ("hold: blue robot on red pad", switch_is_on(red_hold, False, {"blue"}), False),
        ("hold: robot left the pad",   switch_is_on(red_hold, True, set()), False),
        ("latch: first red press",     switch_is_on(red_latch, False, {"red"}), True),
        ("latch: stays on after",      switch_is_on(red_latch, True, set()), True),
        ("latch: blue can't press it", switch_is_on(red_latch, False, {"blue"}), False),
        # old: ("any_of: one on",             door_should_open(any_door, {"S_R": True, "S_B": False}), True),
        ("any_of: one on",             door_should_open(any_door, {"S1": True, "S2": False}), True),
        # old: ("any_of: none on",            door_should_open(any_door, {"S_R": False, "S_B": False}), False),
        ("any_of: none on",            door_should_open(any_door, {"S1": False, "S2": False}), False),
        # old: ("all_of: one on",             door_should_open(all_door, {"S_R": True, "S_B": False}), False),
        ("all_of: one on",             door_should_open(all_door, {"S1": True, "S2": False}), False),
        # old: ("all_of: both on",            door_should_open(all_door, {"S_R": True, "S_B": True}), True),
        ("all_of: both on",            door_should_open(all_door, {"S1": True, "S2": True}), True),
        ("robot status: executor message",
         robot_info_from_status('{"robot1": {"task": "CROSS:D1", "status": {"type": "waiting", '
                                '"repair_time": null, "waiting_for": ["HOLD:S1"]}}}', {"robot1", "robot2"}),
         {"robot1": {"task": "CROSS:D1",
                     "status": {"type": "waiting", "repair_time": None, "waiting_for": ["HOLD:S1"]}}}),
        ("robot status: unknown robot and bad fields",
         robot_info_from_status('{"robot9": {"task": "EXIT"}, "robot2": {"task": 5, "status": "x"}}',
                                {"robot1", "robot2"}),
         {"robot2": {"task": None, "status": {"type": "ok", "repair_time": None, "waiting_for": None}}}),
        ("robot status: not JSON",     robot_info_from_status("nope", {"robot1"}), None),
    ]
    bad = 0
    for name, got, want in cases:
        ok = got == want
        bad += not ok
        print(f"  {'PASS' if ok else 'FAIL'}  {name}: got {got}, want {want}")
    print("ALL PASS" if bad == 0 else f"{bad} FAILED")
    return bad == 0


# ------------------------------------------------------------- Gazebo I/O loop --

def start_world_state_publisher(robot_ids, on_robot_info, log):
    """(publish, spin_once) for the ROS 2 side, or (None, None) if ROS 2 isn't sourced.

    publish(text): publish a JSON string on /team/world_state. The topic is 'transient local':
        a node that subscribes later with transient-local QoS gets the latest state at once. A
        volatile subscriber gets it within HEARTBEAT_PERIOD_S, because run() re-sends the
        latest state at least that often.
    spin_once(): handle waiting /team/robot_status messages (from the executor); each valid one
        is passed to on_robot_info(robot_info).
    """
    try:
        import rclpy
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from std_msgs.msg import String
    except ImportError:
        return None, None
    rclpy.init()
    ros_node = rclpy.create_node("switch_manager")
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = ros_node.create_publisher(String, WORLD_STATE_TOPIC, qos)

    def on_status(msg):
        info = robot_info_from_status(msg.data, robot_ids)
        if info is None:
            log(f"ignoring an invalid {ROBOT_STATUS_TOPIC} message: {msg.data[:80]}")
        else:
            on_robot_info(info)

    # The executor publishes transient local, so the latest status arrives even if it started first.
    ros_node.create_subscription(String, ROBOT_STATUS_TOPIC, on_status, qos)

    def publish(text):
        msg = String()
        msg.data = text
        pub.publish(msg)

    def spin_once():
        rclpy.spin_once(ros_node, timeout_sec=0.0)
    return publish, spin_once


def run(scenario_path):
    from gz.transport13 import Node
    from gz.msgs10.double_pb2 import Double
    from gz.msgs10.pose_v_pb2 import Pose_V

    with open(scenario_path) as f:
        sc = yaml.safe_load(f)
    world = sc["scenario"]["gz_world_name"]
    radius = sc["scenario"]["on_pad_radius_m"]
    robots = {r["id"]: r for r in sc["robots"]}
    switches = {s["id"]: s for s in sc["switches"]}
    doors = sc["doors"]

    poses = {}                                   # robot id -> (x, y), true pose from Gazebo
    switch_on = {sid: False for sid in switches}
    door_open = {d["id"]: False for d in doors}

    def on_poses(msg):                           # Gazebo publishes every model's world pose here
        for p in msg.pose:
            if p.name in robots:                 # model name == robot id (set by robot_name:= at spawn)
                poses[p.name] = (p.position.x, p.position.y)

    node = Node()
    node.subscribe(Pose_V, f"/world/{world}/pose/info", on_poses)
    pubs = {d["id"]: node.advertise(f"/maze/door/{d['id']}/position", Double) for d in doors}
    t0 = time.time()
    log = lambda text: print(f"[{time.time() - t0:7.2f}s] {text}", flush=True)
    log(f"switch manager up; world={world}; watching robots {list(robots)}")

    # World state for the planners (scripts/world_state.py)
    robot_info = {}                              # robot id -> {"task", "status"}, from the executor
    publish_world_state, spin_ros = start_world_state_publisher(set(robots), robot_info.update, log)
    if publish_world_state is None:
        log(f"ROS 2 not sourced: world state is built but not published on {WORLD_STATE_TOPIC}")
    world_state_on = True
    known_rooms = set()
    last_structure = last_full = None   # structure = everything except costs
    last_summary = None                 # known rooms and task ids, for the log
    last_publish_time = 0.0

    while True:
        if set(poses) != set(robots):
            missing = sorted(set(robots) - set(poses))
            log(f"waiting for robot poses: {missing} (spawned with matching robot_name?)")
            time.sleep(1.0)
            continue

        for sid, sw in switches.items():
            # colors = {robots[rid]["color"] for rid, xy in poses.items()
            #           if robot_on_pad(xy, sw, radius)}
            colors = {c for rid, xy in poses.items()
                      if robot_on_pad(xy, sw, radius) for c in robots[rid]["colors"]}
            new = switch_is_on(sw, switch_on[sid], colors)
            if new != switch_on[sid]:
                log(f"switch {sid} ({sw['color']} {sw['mode']}) -> {'ON' if new else 'OFF'}")
            switch_on[sid] = new

        for d in doors:
            should = door_should_open(d, switch_on)
            if should != door_open[d["id"]]:
                log(f"door {d['id']} -> {'OPEN' if should else 'CLOSED'}")
            door_open[d["id"]] = should
            msg = Double()
            msg.data = d["open_joint_position_m"] if should else d["closed_joint_position_m"]
            pubs[d["id"]].publish(msg)           # re-sent every step, so a lost message doesn't matter

        if world_state_on:
            if spin_ros:
                spin_ros()                       # robot tasks/status from the executor, if it runs
            try:
                robot_xy = dict(poses)           # copy: the Gazebo callback updates poses in another thread
                known_rooms = world_state.discover(known_rooms, robot_xy, sc["rooms"])
                state = world_state.build_world_state(sc, known_rooms, robot_xy, switch_on, door_open,
                                                      robot_info)
            except NotImplementedError as e:
                world_state_on = False
                log(f"world state off: {e} is not written yet (scripts/world_state.py)")
            else:
                structure = json.dumps({k: v for k, v in state.items() if k != "costs"})
                full = json.dumps(state)
                now = time.time()
                structure_changed = structure != last_structure
                costs_changed = full != last_full and now - last_publish_time >= COSTS_ONLY_MIN_PERIOD_S
                heartbeat_due = now - last_publish_time >= HEARTBEAT_PERIOD_S
                if structure_changed or costs_changed or heartbeat_due:
                    if publish_world_state:
                        publish_world_state(full)
                    last_full, last_publish_time = full, now
                if structure_changed:
                    last_structure = structure
                    summary = (sorted(known_rooms), [t["id"] for t in state["tasks"]])
                    if summary != last_summary:  # robot task/status changes are published, not logged
                        last_summary = summary
                        log(f"world state: rooms known {summary[0]}; tasks {summary[1]}")

        time.sleep(1.0 / RATE_HZ)


if __name__ == "__main__":
    if len(sys.argv) == 2 and sys.argv[1] == "--selftest":
        sys.exit(0 if selftest() else 1)
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    run(sys.argv[1])
