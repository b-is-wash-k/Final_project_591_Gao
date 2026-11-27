# Robot skills. Every robot has every skill; switch skills also need a matching color.
# A robot runs exactly one skill at a time.
from skills import ACTIVATE, HOLD, CROSS, EXIT, WAIT

# ACTIVATE(robot, switch)  press a LATCH switch once; it stays ON forever. Robot colors must include the switch color.
# HOLD(robot, switch)      stand on a HOLD switch; it is ON only while the robot stays on it.
#                          Robot colors must include the switch color.
# CROSS(robot, door)       drive through `door` into the room on the other side.
#                          The door must stay open for the whole crossing.
#                          This is also how the team explores: if the door's other_room is 'UNKNOWN',
#                          crossing it reveals that room's switches, doors and exit.
# EXIT(robot)              drive to the exit. The mission is done when every working robot is at the exit.
# WAIT(robot)              do nothing until the next planning cycle.

# Door rules:  'open_if': {'any_of': [...]} -> open while at least one listed switch is ON
#              'open_if': {'all_of': [...]} -> open while every listed switch is ON
#              'open_if': None              -> the switches found so far can't open it (none found, or for
#                                              all_of not all found), or it needs no switch; use 'open'
# A CROSS task's 'depends' lists the HOLD tasks that can keep its door open: for any_of ONE of them
# is enough, for all_of ALL are needed. It is left out when no HOLD can help: the door needs no switch,
# only latch switches open it (use ACTIVATE, see 'enables'), not enough of its switches are found yet,
# or latched switches already keep it open.
# Costs never lead through a door that a robot is holding open: get past it with CROSS first.
# Task ids are written SKILL:TARGET, e.g. 'ACTIVATE:S2', 'HOLD:S1', 'CROSS:D1', plus 'EXIT' and 'WAIT'.
# Rooms behind a door to 'UNKNOWN' have no name yet; explore them with CROSS:<door>.
# costs[robot][task] is the path length in meters on the known map. A missing entry means unreachable.
