"""Plan validator: the rules of proposal section 3.4.

validate_plan(plan, state) returns a list of human-readable violations;
an empty list means the plan is valid. `plan` is {"robot1": "HOLD:S1", ...}
and `state` is the planner-input JSON from docs/communication_standards.md.
"""
from collections import Counter

ALWAYS_ALLOWED = {'WAIT'}          # needs no entry in the task menu or the cost matrix
SHAREABLE = {'EXIT', 'WAIT'}       # may be given to several robots at once
SWITCH_TASKS = {'ACTIVATE', 'HOLD'}


def validate_plan(plan, state):
    robots = {r['id']: r for r in state.get('robots', [])}
    tasks = {t['id']: t for t in state.get('tasks', [])}
    switches = {s['id']: s for s in state.get('switches', [])}
    doors = {d['id']: d for d in state.get('doors', [])}
    costs = state.get('costs', {})
    errors = []

    for rid in plan:
        if rid not in robots:
            errors.append(f'unknown robot id {rid!r}')
    for rid in robots:
        if rid not in plan:
            errors.append(f'{rid} has no task (use "WAIT" for an idle robot)')

    for rid, tid in plan.items():
        if rid not in robots or tid in ALWAYS_ALLOWED:
            continue
        task = tasks.get(tid)
        if task is None:
            errors.append(f'{rid}: unknown task id {tid!r}')
            continue
        color = task.get('needs_color')
        if task['type'] in SWITCH_TASKS and color and color not in robots[rid].get('colors', []):
            errors.append(f'{rid}: {tid} needs a {color} robot, {rid} is {robots[rid].get("colors")}')
        if tid not in costs.get(rid, {}):
            errors.append(f'{rid}: {tid} is unreachable for {rid} (no cost entry)')

    for tid, n in Counter(plan.values()).items():
        if n > 1 and tid not in SHAREABLE:
            errors.append(f'{tid} is assigned to {n} robots; only EXIT and WAIT may be shared')

    # A crossing needs its door to stay open: latched switches, or switches this plan holds/activates.
    latched = {sid for sid, s in switches.items() if s.get('mode') == 'latch' and s.get('active')}
    pressed = {tasks[t]['target'] for t in plan.values()
               if t in tasks and tasks[t]['type'] in SWITCH_TASKS}
    for rid, tid in plan.items():
        task = tasks.get(tid)
        if not task or task['type'] != 'CROSS':
            continue
        door = doors.get(task['target'])
        if door is None:
            errors.append(f'{rid}: {tid} refers to unknown door {task["target"]!r}')
        elif not _door_stays_open(door, latched | pressed):
            errors.append(f'{rid}: {tid} but door {door["id"]} is closed (or will close) and no robot '
                          f'is assigned to hold or activate a switch that opens it')
    return errors


def _door_stays_open(door, switches_on):
    rule = door.get('open_if')
    if not rule:
        return bool(door.get('open'))
    if 'all_of' in rule:
        return all(s in switches_on for s in rule['all_of'])
    return any(s in switches_on for s in rule.get('any_of', []))
