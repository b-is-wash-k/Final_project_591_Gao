# ============================================================== EXAMPLE 1 ==
robots = [
    {'id': 'robot1', 'colors': ['red'], 'status': {'type': 'ok'}, 'task': None},
    {'id': 'robot2', 'colors': ['blue'], 'status': {'type': 'ok'}, 'task': None},
]
costs = {
    'robot1': {'EXPLORE:1,2': 2.5, 'EXPLORE:2,1': 6.0},
    'robot2': {'EXPLORE:1,2': 5.5, 'EXPLORE:2,1': 2.0},
}
# GENERAL TASK DECOMPOSITION
# Independent subtasks:
# SubTask 1: Explore room 1,2. (Skills: EXPLORE)
# SubTask 2: Explore room 2,1. (Skills: EXPLORE)
# Order: SubTask 1 and SubTask 2 in parallel. Replan when something new is found.

# SOLUTION
# All the robots have the skills needed for SubTask 1 and SubTask 2 (EXPLORE needs no color).
# SubTask 1 and SubTask 2 run in parallel, so they need two different robots.
# robot1 is closer to room 1,2 (2.5 m vs 5.5 m). robot2 is closer to room 2,1 (2.0 m vs 6.0 m).
# For SubTask 1 assign robot1.
# For SubTask 2 assign robot2.
# No robot teams are needed.


# ============================================================== EXAMPLE 2 ==
robots = [
    {'id': 'robot1', 'colors': ['red'], 'status': {'type': 'ok'}, 'task': None},
    {'id': 'robot2', 'colors': ['blue'], 'status': {'type': 'ok'}, 'task': None},
    {'id': 'robot3', 'colors': ['red', 'blue'], 'status': {'type': 'ok'}, 'task': None},
]
costs = {
    'robot1': {'ACTIVATE:S2': 3.0, 'ACTIVATE:S3': 4.5, 'CROSS:D2': 4.0},
    'robot2': {'ACTIVATE:S2': 4.0, 'ACTIVATE:S3': 2.0, 'CROSS:D2': 3.5},
    'robot3': {'ACTIVATE:S2': 1.5, 'ACTIVATE:S3': 4.0, 'CROSS:D2': 2.0},
}
# GENERAL TASK DECOMPOSITION
# Independent subtasks:
# SubTask 1: Activate red latch switch S2. (Skills: ACTIVATE, color red)
# SubTask 2: Activate blue latch switch S3. (Skills: ACTIVATE, color blue)
# SubTask 3: Cross D2. (Skills: CROSS) Every robot does this after SubTask 1 and SubTask 2.
# Order: SubTask 1 and SubTask 2 in parallel, then SubTask 3.

# SOLUTION
# SubTask 1 needs a red robot: robot1 (red) or robot3 (red, blue).
# SubTask 2 needs a blue robot: robot2 (blue) or robot3 (red, blue).
# SubTask 1 and SubTask 2 run in parallel, so they need two different robots.
# Option A: robot1 -> S2 (3.0 m), robot2 -> S3 (2.0 m); both done after 3.0 m.
# Option B: robot3 -> S2 (1.5 m), robot2 -> S3 (2.0 m); both done after 2.0 m.
# Option B finishes sooner.
# For SubTask 1 assign robot3.
# For SubTask 2 assign robot2.
# robot1 has no switch to press. SubTask 3 cannot start until S2 and S3 are both ON, so robot1 waits.
# SubTask 3 is for every robot once D2 is open.
# No robot teams are needed: latch switches stay ON after one press.


# ============================================================== EXAMPLE 3 ==
robots = [
    {'id': 'robot1', 'colors': ['red'], 'status': {'type': 'ok'}, 'task': 'EXPLORE:1,1'},
    {'id': 'robot2', 'colors': ['blue'], 'status': {'type': 'ok'}, 'task': None},
    {'id': 'robot3', 'colors': ['green'], 'status': {'type': 'ok'}, 'task': None},
]
costs = {
    'robot1': {'HOLD:S4': 2.0, 'CROSS:D3': 3.0, 'EXPLORE:2,1': 5.0},
    'robot2': {'HOLD:S4': 3.5, 'CROSS:D3': 2.5, 'EXPLORE:2,1': 4.0},
    'robot3': {'HOLD:S4': 4.0, 'CROSS:D3': 6.0, 'EXPLORE:2,1': 1.5},
}
# GENERAL TASK DECOMPOSITION
# Independent subtasks:
# SubTask 1: Hold red hold switch S4 to open D3. (Skills: HOLD, color red) Keep holding until SubTask 2 is done.
# SubTask 2: Cross D3 while S4 is held. (Skills: CROSS) Must run at the same time as SubTask 1.
# SubTask 3: Explore room 2,1. (Skills: EXPLORE) Independent of SubTask 1 and SubTask 2.
# Order: SubTask 1, SubTask 2 and SubTask 3 in parallel; SubTask 1 ends only after SubTask 2 ends.

# SOLUTION
# SubTask 1 needs a red robot. Only robot1 is red, so robot1 must hold S4.
# SubTask 2 needs D3 open for the whole crossing. D3 is open only while S4 is held,
# so SubTask 1 and SubTask 2 must run at the same time on two different robots.
# robot1 is busy with SubTask 1. robot2 (2.5 m) is closer to D3 than robot3 (6.0 m).
# For SubTask 1 assign robot1.
# For SubTask 2 assign robot2.
# SubTask 3 needs only EXPLORE. robot3 is free and closest to room 2,1 (1.5 m).
# For SubTask 3 assign robot3.
# Team: robot1 (HOLD S4) + robot2 (CROSS D3). robot1 keeps holding until robot2 is through D3.


# ============================================================== EXAMPLE 4 ==
robots = [
    {'id': 'robot1', 'colors': ['red'], 'status': {'type': 'ok'}, 'task': 'CROSS:D5'},
    {'id': 'robot2', 'colors': ['blue'], 'status': {'type': 'ok'}, 'task': None},
    {'id': 'robot3', 'colors': ['red'], 'status': {'type': 'broken', 'repair_time': 30}, 'task': None},
]
costs = {
    'robot1': {'EXIT': 3.0},
    'robot2': {'EXIT': 1.0},
    'robot3': {'EXIT': 4.0},
}
# GENERAL TASK DECOMPOSITION
# Independent subtasks:
# SubTask 1: Every working robot goes to the exit. (Skills: EXIT)
# Order: SubTask 1 for all robots at once.

# SOLUTION
# All the robots have the EXIT skill and every robot has a cost for EXIT, so all can reach it.
# robot3 is broken for the next 30 s and cannot move now.
# For SubTask 1 assign robot1 and robot2 (EXIT may be given to several robots).
# robot3 waits until it is repaired; the next plan will send it to the exit.
# No robot teams are needed.
