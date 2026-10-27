"""Deterministic per-type namespaces for the existing robot launch list."""
import re


def robot_namespaces(robots):
    """Strings use their name as type; objects supply distinct name/type values.

    Sorting Webots names makes numbering independent of launch-list order.
    Types that normalize identically share a counter, preventing collisions.
    """
    if not isinstance(robots, list):
        raise ValueError('robots must be a JSON list of names or {name, type} objects')
    types = {}
    for robot in robots:
        if isinstance(robot, str):
            name = kind = robot
        elif isinstance(robot, dict):
            name, kind = robot.get('name'), robot.get('type')
        else:
            raise ValueError('Each robot must be a name or a {name, type} object')
        if not isinstance(name, str) or not name or not isinstance(kind, str) or not kind.strip():
            raise ValueError('Each robot needs a nonempty name and type')
        if name in types:
            raise ValueError('Webots Robot names must be unique for external controllers')
        kind = re.sub('[^a-z0-9_]', '_', kind.lower()).strip('_') or 'robot'
        if kind[0].isdigit():
            kind = 'robot_' + kind
        types[name] = kind
    counts = {}
    namespaces = {}
    for name in sorted(types):
        kind = types[name]
        counts[kind] = counts.get(kind, 0) + 1
        namespaces[name] = f'{kind}_{counts[kind]}'
    return namespaces
