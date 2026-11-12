#!/usr/bin/env python3
"""Switch manager: robot on a pad -> switch state -> gate open/closed.

Reads the true robot positions from Gazebo (not odometry), decides which
switches are ON and which doors are OPEN, and sends each gate's target
position to the Gazebo topic /maze/door/<ID>/position.

    robot poses (Gazebo) -> switch manager -> gate controller -> gate moves

Usage (from the repository root; use the system python3):
    python3 scripts/switch_manager.py --selftest            # check the rules, no Gazebo needed
    python3 scripts/switch_manager.py config/scenario.yaml  # run with Gazebo
"""
import math
import sys
import time

import yaml

RATE_HZ = 10


# -------------------------------------------------------------- puzzle rules --

def robot_on_pad(robot_xy, switch, radius):
    """True if the robot's center is on this switch pad.

    robot_xy: (x, y) of the robot in meters, world frame.
    switch:   one entry of `switches` in the scenario, e.g.
              {"id": "S_R", "color": "red", "mode": "hold", "pose": [-1.1, -1.45, 0.0]}
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

    door:      one entry of `doors`, e.g. {"id": "D1", "logic": "any_of", "requires": ["S_R", "S_B"], ...}
    switch_on: dict switch id -> True/False, e.g. {"S_R": True, "S_B": False}
    Rules: any_of = at least one required switch ON; all_of = every required switch ON.
    """
    required_states = [switch_on[sid] for sid in door["requires"]]

    if door["logic"] == "any_of":
        return any(required_states)
    if door["logic"] == "all_of":
        return all(required_states)

    raise ValueError(f"Unknown door logic: {door['logic']}")


# ------------------------------------------------------- self-test (no Gazebo) --

def selftest():
    red_hold = {"id": "S_R", "color": "red", "mode": "hold", "pose": [-1.1, -1.45, 0.0]}
    red_latch = {"id": "S_L", "color": "red", "mode": "latch", "pose": [0.0, 0.0, 0.0]}
    any_door = {"id": "D1", "logic": "any_of", "requires": ["S_R", "S_B"]}
    all_door = {"id": "D2", "logic": "all_of", "requires": ["S_R", "S_B"]}
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
        ("any_of: one on",             door_should_open(any_door, {"S_R": True, "S_B": False}), True),
        ("any_of: none on",            door_should_open(any_door, {"S_R": False, "S_B": False}), False),
        ("all_of: one on",             door_should_open(all_door, {"S_R": True, "S_B": False}), False),
        ("all_of: both on",            door_should_open(all_door, {"S_R": True, "S_B": True}), True),
    ]
    bad = 0
    for name, got, want in cases:
        ok = got == want
        bad += not ok
        print(f"  {'PASS' if ok else 'FAIL'}  {name}: got {got}, want {want}")
    print("ALL PASS" if bad == 0 else f"{bad} FAILED")
    return bad == 0


# ------------------------------------------------------------- Gazebo I/O loop --

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

    while True:
        if set(poses) != set(robots):
            missing = sorted(set(robots) - set(poses))
            log(f"waiting for robot poses: {missing} (spawned with matching robot_name?)")
            time.sleep(1.0)
            continue

        for sid, sw in switches.items():
            colors = {robots[rid]["color"] for rid, xy in poses.items()
                      if robot_on_pad(xy, sw, radius)}
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

        time.sleep(1.0 / RATE_HZ)


if __name__ == "__main__":
    if len(sys.argv) == 2 and sys.argv[1] == "--selftest":
        sys.exit(0 if selftest() else 1)
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    run(sys.argv[1])
