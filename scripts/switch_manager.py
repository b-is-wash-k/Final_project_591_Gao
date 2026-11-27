#!/usr/bin/env python3
"""Switch manager: robot on a pad -> switch state -> gate open/closed.

Reads the true robot positions from Gazebo (not odometry), decides which
switches are ON and which doors are OPEN, and sends each gate's target
position to the Gazebo topic /maze/door/<ID>/position.

    robot poses (Gazebo) -> switch manager -> gate controller -> gate moves

It also builds the planners' world state (scripts/world_state.py) and publishes
it as JSON on the ROS 2 topic /team/world_state.

Usage (from the repository root; use the system python3):
    python3 scripts/switch_manager.py --selftest            # check the rules, no Gazebo needed
    python3 scripts/switch_manager.py config/scenario.yaml  # run with Gazebo
"""
import json
import math
import sys
import time
import traceback

import yaml

import world_state

RATE_HZ = 10
WORLD_STATE_TOPIC = "/team/world_state"
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
    A door that requires no switch at all is always open (a plain passage).
    """
    if not door["requires"]:
        return True
    required_states = [switch_on[sid] for sid in door["requires"]]

    if door["logic"] == "any_of":
        return any(required_states)
    if door["logic"] == "all_of":
        return all(required_states)

    raise ValueError(f"Unknown door logic: {door['logic']}")


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
        ("no switch needed: open",     door_should_open({"id": "D3", "logic": "any_of", "requires": []}, {}), True),
    ]
    bad = 0
    for name, got, want in cases:
        ok = got == want
        bad += not ok
        print(f"  {'PASS' if ok else 'FAIL'}  {name}: got {got}, want {want}")
    print("ALL PASS" if bad == 0 else f"{bad} FAILED")
    return bad == 0


# ------------------------------------------------------------- Gazebo I/O loop --

def start_world_state_publisher():
    """A function that publishes a JSON string on /team/world_state, or None if ROS 2 isn't sourced.

    The topic is 'transient local': a node that subscribes later with transient-local QoS gets
    the latest state at once. A volatile subscriber gets it within HEARTBEAT_PERIOD_S, because
    run() re-sends the latest state at least that often.
    """
    try:
        import rclpy
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from rclpy.signals import SignalHandlerOptions
        from std_msgs.msg import String
    except ImportError:
        return None
    # No rclpy signal handlers: Ctrl+C / SIGTERM (from run.sh) stop this script directly,
    # instead of invalidating the ROS context and failing the next publish.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    ros_node = rclpy.create_node("switch_manager")
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = ros_node.create_publisher(String, WORLD_STATE_TOPIC, qos)

    def publish(text):
        msg = String()
        msg.data = text
        pub.publish(msg)
    return publish


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
    publish_world_state = start_world_state_publisher()
    if publish_world_state is None:
        log(f"ROS 2 not sourced: world state is built but not published on {WORLD_STATE_TOPIC}")
    problems = world_state.check_scenario(sc)
    for problem in problems:
        log(f"scenario problem: {problem}")
    world_state_on = not problems
    if problems:
        log("world state off until the scenario is fixed; switches and doors keep working")
    known_rooms = set()
    last_structure = last_full = None   # structure = everything except costs
    last_summary = None                 # known rooms + task ids, for the log line
    last_publish_time = -math.inf       # time.monotonic() of the last publish

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
            try:
                robot_xy = dict(poses)           # copy: the Gazebo callback updates poses in another thread
                known_rooms = world_state.discover(known_rooms, robot_xy, sc["rooms"])
                state = world_state.build_world_state(sc, known_rooms, robot_xy, switch_on, door_open)
            except NotImplementedError as e:
                world_state_on = False
                log(f"world state off: {e} is not written yet (scripts/world_state.py)")
            except Exception:              # never let a world-state bug stop the gates
                world_state_on = False
                log("world state off after an error; switches and doors keep working:\n"
                    + traceback.format_exc())
            else:
                structure = json.dumps({k: v for k, v in state.items() if k != "costs"})
                full = json.dumps(state)
                now = time.monotonic()
                structure_changed = structure != last_structure
                costs_changed = full != last_full and now - last_publish_time >= COSTS_ONLY_MIN_PERIOD_S
                heartbeat_due = now - last_publish_time >= HEARTBEAT_PERIOD_S
                if structure_changed or costs_changed or heartbeat_due:
                    if publish_world_state:
                        publish_world_state(full)
                    last_full, last_publish_time = full, now
                if structure_changed:
                    last_structure = structure
                summary = (sorted(known_rooms, key=world_state.id_key), [t["id"] for t in state["tasks"]])
                if summary != last_summary:
                    last_summary = summary
                    log(f"world state: rooms known {summary[0]}; tasks {summary[1]}")

        time.sleep(1.0 / RATE_HZ)


if __name__ == "__main__":
    if len(sys.argv) == 2 and sys.argv[1] == "--selftest":
        sys.exit(0 if selftest() else 1)
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    run(sys.argv[1])
