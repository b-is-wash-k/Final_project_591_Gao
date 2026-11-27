# Each robot runs exactly ONE task at a time, and the planner is called again whenever the
# world changes (something is found, a switch or door changes, a task ends, a robot fails).
# So the allocation only says what every robot must do NOW, as JSON:
#   {"assignments": {"<robot id>": "<task id>", ...}, "reasons": {"<robot id>": "<short reason>", ...}}
# Rules:
#   - Every robot gets exactly one task id from `tasks`, or "WAIT".
#   - A task id may be given to only one robot, except "EXIT" and "WAIT".
#   - A robot may only ACTIVATE or HOLD a switch whose color is in its colors.
#   - A robot may only get a task that has an entry in costs[robot] ("WAIT" is always allowed).
#   - CROSS a closed door only if, in the same allocation, other robots HOLD or ACTIVATE switches that open it.
#   - A robot already holding a switch keeps "HOLD:<S>" while another robot still depends on it.
#   - A broken robot gets "WAIT".

# ============================================================== EXAMPLE 1 ==
robots = ['robot1', 'robot2']
tasks = ['CROSS:D1', 'CROSS:D2']
# SOLUTION
# For SubTask 1 (CROSS D1, explores the room behind it) assign robot1.
# For SubTask 2 (CROSS D2, explores the room behind it) assign robot2.
# No robot teams are needed.

# ALLOCATION
{"assignments": {"robot1": "CROSS:D1", "robot2": "CROSS:D2"},
 "reasons": {"robot1": "SubTask 1; closest to D1, explores behind it",
             "robot2": "SubTask 2; closest to D2, explores behind it"}}


# ============================================================== EXAMPLE 2 ==
robots = ['robot1', 'robot2', 'robot3']
tasks = ['ACTIVATE:S2', 'ACTIVATE:S3', 'CROSS:D2']
# SOLUTION
# For SubTask 1 (ACTIVATE S2, red) assign robot3.
# For SubTask 2 (ACTIVATE S3, blue) assign robot2.
# robot1 waits: SubTask 3 (CROSS D2) cannot start until S2 and S3 are both ON.
# No robot teams are needed.

# ALLOCATION
{"assignments": {"robot1": "WAIT", "robot2": "ACTIVATE:S3", "robot3": "ACTIVATE:S2"},
 "reasons": {"robot1": "nothing useful until D2 opens", "robot2": "SubTask 2; blue",
             "robot3": "SubTask 1; red and closest to S2"}}


# ============================================================== EXAMPLE 3 ==
robots = ['robot1', 'robot2', 'robot3']
tasks = ['HOLD:S4', 'CROSS:D3', 'CROSS:D4']
# SOLUTION
# For SubTask 1 (HOLD S4, red) assign robot1.
# For SubTask 2 (CROSS D3) assign robot2.
# For SubTask 3 (CROSS D4, explores the room behind it) assign robot3.
# Team: robot1 (HOLD S4) + robot2 (CROSS D3). robot1 keeps holding until robot2 is through D3.

# ALLOCATION
{"assignments": {"robot1": "HOLD:S4", "robot2": "CROSS:D3", "robot3": "CROSS:D4"},
 "reasons": {"robot1": "SubTask 1; only red robot, holds D3 open",
             "robot2": "SubTask 2; crosses while robot1 holds S4",
             "robot3": "SubTask 3; free and closest to D4, explores behind it"}}


# ============================================================== EXAMPLE 4 ==
robots = ['robot1', 'robot2', 'robot3']
tasks = ['EXIT']
# SOLUTION
# For SubTask 1 (EXIT) assign robot1 and robot2.
# robot3 is broken and waits until it is repaired.
# No robot teams are needed.

# ALLOCATION
{"assignments": {"robot1": "EXIT", "robot2": "EXIT", "robot3": "WAIT"},
 "reasons": {"robot1": "exit reachable", "robot2": "exit reachable", "robot3": "broken"}}
