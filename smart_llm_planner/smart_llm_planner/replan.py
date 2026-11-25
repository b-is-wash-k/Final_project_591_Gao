"""When to replan: compare two consecutive world states."""


def replan_triggers(prev, cur):
    """Reasons to replan when the world state goes from `prev` to `cur` (empty list: no need)."""
    if prev is None:
        return ['initial world state']

    def by_id(state, key):
        return {x['id']: x for x in state.get(key, [])}

    triggers = []
    for key, noun in (('rooms', 'room'), ('switches', 'switch'), ('doors', 'door'), ('tasks', 'task')):
        old, new = by_id(prev, key), by_id(cur, key)
        triggers += [f'found {noun} {i}' for i in sorted(new.keys() - old.keys())]
        triggers += [f'{noun} {i} removed' for i in sorted(old.keys() - new.keys())]

    for key, noun, flag in (('switches', 'switch', 'active'), ('doors', 'door', 'open')):
        old = by_id(prev, key)
        for oid, obj in by_id(cur, key).items():
            if oid in old and bool(old[oid].get(flag)) != bool(obj.get(flag)):
                triggers.append(f'{noun} {oid} {flag} -> {bool(obj.get(flag))}')

    old_robots = by_id(prev, 'robots')
    for rid, robot in by_id(cur, 'robots').items():
        old = old_robots.get(rid)
        if old is None:
            triggers.append(f'new robot {rid}')
            continue
        old_status = (old.get('status') or {}).get('type')
        new_status = (robot.get('status') or {}).get('type')
        if old_status != new_status:
            triggers.append(f'{rid} status {old_status} -> {new_status}')
        # A task that ends (task -> null) is an event; our own new assignment is not.
        if old.get('task') is not None and robot.get('task') is None:
            triggers.append(f'{rid} finished {old["task"]}')
    return triggers
