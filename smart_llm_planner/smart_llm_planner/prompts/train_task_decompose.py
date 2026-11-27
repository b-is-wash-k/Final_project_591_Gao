# ============================================================== EXAMPLE 1 ==
# Task Description: All robots must reach the exit.
rooms = [
    {'id': '1,1', 'robots': ['robot1', 'robot2'], 'switches': [],
     'doors': [{'id': 'D1', 'other_room': 'UNKNOWN'}, {'id': 'D2', 'other_room': 'UNKNOWN'}], 'exit': False},
]
switches = []
doors = [
    {'id': 'D1', 'open': True, 'open_if': None},
    {'id': 'D2', 'open': True, 'open_if': None},
]
tasks = [
    {'id': 'CROSS:D1', 'type': 'CROSS', 'target': 'D1', 'depends': None},
    {'id': 'CROSS:D2', 'type': 'CROSS', 'target': 'D2', 'depends': None},
]

# GENERAL TASK DECOMPOSITION
# The exit is not known yet, so the team must explore first.
# D1 and D2 are open and lead to UNKNOWN rooms. Crossing a door into an UNKNOWN room
# explores that room: its switches, doors and exit become known.
# Independent subtasks:
# SubTask 1: Cross D1 to explore the room behind it. (Skills: CROSS)
# SubTask 2: Cross D2 to explore the room behind it. (Skills: CROSS)
# Order: SubTask 1 and SubTask 2 in parallel. Replan when something new is found.

# CODE
def explore_behind_d1():
    # 0: SubTask 1: Cross D1 to explore the room behind it
    # 1: Drive through D1 into the unknown room and reveal it.
    CROSS('D1')

def explore_behind_d2():
    # 0: SubTask 2: Cross D2 to explore the room behind it
    # 1: Drive through D2 into the unknown room and reveal it.
    CROSS('D2')

# Parallelize SubTask 1 and SubTask 2
t1 = threading.Thread(target=explore_behind_d1)
t2 = threading.Thread(target=explore_behind_d2)
# Start executing SubTask 1 and SubTask 2 in parallel
t1.start()
t2.start()
# Wait for both SubTask 1 and SubTask 2 to finish
t1.join()
t2.join()


# ============================================================== EXAMPLE 2 ==
# Task Description: All robots must reach the exit.
rooms = [
    {'id': '1,1', 'robots': ['robot1', 'robot2', 'robot3'], 'switches': ['S2', 'S3'],
     'doors': [{'id': 'D2', 'other_room': 'UNKNOWN'}], 'exit': False},
]
switches = [
    {'id': 'S2', 'color': 'red', 'mode': 'latch', 'active': False, 'controls': 'D2'},
    {'id': 'S3', 'color': 'blue', 'mode': 'latch', 'active': False, 'controls': 'D2'},
]
doors = [
    {'id': 'D2', 'open': False, 'open_if': {'all_of': ['S2', 'S3']}},
]
tasks = [
    {'id': 'ACTIVATE:S2', 'type': 'ACTIVATE', 'target': 'S2', 'needs_color': 'red', 'enables': 'CROSS:D2'},
    {'id': 'ACTIVATE:S3', 'type': 'ACTIVATE', 'target': 'S3', 'needs_color': 'blue', 'enables': 'CROSS:D2'},
    {'id': 'CROSS:D2', 'type': 'CROSS', 'target': 'D2', 'depends': ['ACTIVATE:S2', 'ACTIVATE:S3']},
]

# GENERAL TASK DECOMPOSITION
# The only way forward is D2. D2 opens when BOTH S2 and S3 are ON (all_of).
# S2 and S3 are latch switches: one press each and they stay ON, so nobody has to stay behind.
# Independent subtasks:
# SubTask 1: Activate red latch switch S2. (Skills: ACTIVATE, color red)
# SubTask 2: Activate blue latch switch S3. (Skills: ACTIVATE, color blue)
# SubTask 3: Cross D2. (Skills: CROSS) Every robot does this after SubTask 1 and SubTask 2.
# Order: SubTask 1 and SubTask 2 in parallel, then SubTask 3.

# CODE
def activate_s2():
    # 0: SubTask 1: Activate red latch switch S2
    # 1: Drive onto S2 once.
    ACTIVATE('S2')

def activate_s3():
    # 0: SubTask 2: Activate blue latch switch S3
    # 1: Drive onto S3 once.
    ACTIVATE('S3')

def cross_d2():
    # 0: SubTask 3: Cross D2
    # 1: Drive through D2 (it stays open because S2 and S3 are latched).
    CROSS('D2')

# Parallelize SubTask 1 and SubTask 2
t1 = threading.Thread(target=activate_s2)
t2 = threading.Thread(target=activate_s3)
t1.start()
t2.start()
t1.join()
t2.join()
# Execute SubTask 3 after SubTask 1 and SubTask 2 are complete
cross_d2()


# ============================================================== EXAMPLE 3 ==
# Task Description: All robots must reach the exit.
rooms = [
    {'id': '1,1', 'robots': ['robot1', 'robot2', 'robot3'], 'switches': ['S4'],
     'doors': [{'id': 'D3', 'other_room': 'UNKNOWN'}, {'id': 'D4', 'other_room': 'UNKNOWN'}], 'exit': False},
]
switches = [
    {'id': 'S4', 'color': 'red', 'mode': 'hold', 'active': False, 'controls': 'D3'},
]
doors = [
    {'id': 'D3', 'open': False, 'open_if': {'any_of': ['S4']}},
    {'id': 'D4', 'open': True, 'open_if': None},
]
tasks = [
    {'id': 'HOLD:S4', 'type': 'HOLD', 'target': 'S4', 'needs_color': 'red', 'enables': 'CROSS:D3'},
    {'id': 'CROSS:D3', 'type': 'CROSS', 'target': 'D3', 'depends': ['HOLD:S4']},
    {'id': 'CROSS:D4', 'type': 'CROSS', 'target': 'D4', 'depends': None},
]

# GENERAL TASK DECOMPOSITION
# D3 opens while S4 is ON (any_of). Other switches for D3 may be in rooms not found yet.
# S4 is a hold switch: D3 is open only while a robot stands on S4, so the holder cannot cross D3 itself.
# A robot that crosses D3 explores the room behind it; a switch found there could later hold D3 open for the holder.
# D4 is open and leads to an UNKNOWN room, which may contain the exit. Crossing D4 explores that room.
# Independent subtasks:
# SubTask 1: Hold red hold switch S4 to open D3. (Skills: HOLD, color red) Keep holding until SubTask 2 is done.
# SubTask 2: Cross D3 while S4 is held. (Skills: CROSS) Must run at the same time as SubTask 1.
# SubTask 3: Cross D4 to explore the room behind it. (Skills: CROSS) Independent of SubTask 1 and SubTask 2.
# Order: SubTask 1, SubTask 2 and SubTask 3 in parallel; SubTask 1 ends only after SubTask 2 ends.

# CODE
def hold_s4():
    # 0: SubTask 1: Hold red hold switch S4
    # 1: Stand on S4 until the crossing robot is through D3.
    HOLD('S4')

def cross_d3():
    # 0: SubTask 2: Cross D3 while S4 is held
    # 1: Drive through D3.
    CROSS('D3')

def explore_behind_d4():
    # 0: SubTask 3: Cross D4 to explore the room behind it
    # 1: Drive through D4 into the unknown room and reveal it.
    CROSS('D4')

# Parallelize SubTask 1, SubTask 2 and SubTask 3
t1 = threading.Thread(target=hold_s4)
t2 = threading.Thread(target=cross_d3)
t3 = threading.Thread(target=explore_behind_d4)
t1.start()
t2.start()
t3.start()
# SubTask 1 must not end before SubTask 2 has finished crossing
t2.join()
t1.join()
t3.join()


# ============================================================== EXAMPLE 4 ==
# Task Description: All robots must reach the exit.
rooms = [
    {'id': '1,1', 'robots': [], 'switches': ['S6'], 'doors': [{'id': 'D5', 'other_room': '1,2'}], 'exit': False},
    {'id': '1,2', 'robots': ['robot1', 'robot2', 'robot3'], 'switches': [],
     'doors': [{'id': 'D5', 'other_room': '1,1'}], 'exit': True},
]
switches = [
    {'id': 'S6', 'color': 'blue', 'mode': 'latch', 'active': True, 'controls': 'D5'},
]
doors = [
    {'id': 'D5', 'open': True, 'open_if': {'any_of': ['S6']}},
]
tasks = [
    {'id': 'EXIT', 'type': 'EXIT', 'target': '1,2'},
]

# GENERAL TASK DECOMPOSITION
# The exit is in room 1,2 and every robot is already in room 1,2. Nothing blocks the way.
# Independent subtasks:
# SubTask 1: Every working robot goes to the exit. (Skills: EXIT)
# Order: SubTask 1 for all robots at once.

# CODE
def exit_maze():
    # 0: SubTask 1: Go to the exit
    # 1: Drive to the exit and stop there.
    EXIT()

exit_maze()
