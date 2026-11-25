# Robot skills. Every robot has every skill; switch skills also need a matching color.
# A robot runs exactly one skill at a time.
from skills import EXPLORE, ACTIVATE, HOLD, CROSS, EXIT, WAIT

# EXPLORE(robot, room)     drive into the unexplored room `room` and reveal the switches, doors and exit there.
# ACTIVATE(robot, switch)  press a LATCH switch once; it stays ON forever. Robot colors must include the switch color.
# HOLD(robot, switch)      stand on a HOLD switch; it is ON only while the robot stays on it.
#                          Robot colors must include the switch color.
# CROSS(robot, door)       drive through `door` into the room on the other side.
#                          The door must stay open for the whole crossing.
# EXIT(robot)              drive to the exit. The mission is done when every working robot is at the exit.
# WAIT(robot)              do nothing until the next planning cycle.

# Door rules:  'open_if': {'any_of': [...]} -> open while at least one listed switch is ON
#              'open_if': {'all_of': [...]} -> open while every listed switch is ON
#              'open_if': None              -> the controlling switches have not been found yet
# Task ids are written SKILL:TARGET, e.g. 'HOLD:S1', 'CROSS:D1', 'EXPLORE:1,2', plus 'EXIT' and 'WAIT'.
# costs[robot][task] is the path length in meters on the known map. A missing entry means unreachable.
