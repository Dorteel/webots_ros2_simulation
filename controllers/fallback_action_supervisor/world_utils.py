"""
Contents
--------
walk_nodes()           - Walks a node and its exposed descendants.
get_node()             - Retrieves a Webots node by its name field.
get_descendant()       - Retrieves a named node below another node.
get_world_position()   - Returns a Pose node's world position.
get_slot_connector()   - Retrieves a Connector inside a robot slot PROTO.
get_object_connector() - Retrieves a Connector inside an object PROTO.
validate_coordinates() - Validates an x, y, z coordinate sequence.
teleport_node()        - Moves a node directly to world coordinates.
set_yaw()              - Rotates a node around the world vertical axis.
find_clear_pose()      - Finds a collision-conscious pose near a target.
set_hinge_position()   - Sets a HingeJoint position in radians.
"""

from math import atan2, cos, hypot, isfinite, pi, sin


ROBOT_RADIUS = 0.35
DEFAULT_OBSTACLE_RADIUS = 0.25


def walk_nodes(root):
    """Yield a node and all nodes reachable through its exposed fields."""
    pending = [root]
    visited = set()
    while pending:
        node = pending.pop()
        node_id = node.getId()
        if node_id in visited:
            continue
        visited.add(node_id)
        yield node

        for index in range(node.getNumberOfFields()):
            field = node.getFieldByIndex(index)
            if field.getTypeName() == "SFNode":
                child = field.getSFNode()
                if child is not None:
                    pending.append(child)
            elif field.getTypeName() == "MFNode":
                pending.extend(field.getMFNode(i) for i in range(field.getCount()))


def get_node(supervisor, name, kind="node"):
    """Return the uniquely named node found in the complete scene tree."""
    return get_descendant(supervisor.getRoot(), name, kind)


def get_descendant(root, name, kind="node"):
    """Return the uniquely named node within a scene-tree branch."""
    if not isinstance(name, str) or not name.strip():
        raise ValueError(f"{kind} name must be a non-empty string")

    matches = []
    for node in walk_nodes(root):
        name_field = node.getField("name")
        if name_field is not None and name_field.getSFString() == name:
            matches.append(node)

    if not matches:
        raise ValueError(f"{kind} not found by name: {name}")
    if len(matches) > 1:
        raise ValueError(f"{kind} name is not unique: {name}")
    return matches[0]


def get_world_position(node):
    """Return a Pose node's world position as three floats."""
    try:
        return list(node.getPosition())
    except Exception:
        raise ValueError("node has no world position") from None


def get_slot_connector(robot, field_name):
    """Return a connector-enabled end-effector PROTO."""
    field = robot.getField(field_name)
    endpoint = field and field.getSFNode()
    if endpoint is None:
        raise ValueError(f"robot has no {field_name} endpoint")
    if endpoint.getField("connectorModel") is None or endpoint.getField("connectorLocked") is None:
        raise ValueError(f"{field_name} has no fallback Connector")
    return endpoint


def get_object_connector(node):
    """Return a connector-enabled object PROTO."""
    if node.getField("connectorModel") is None:
        raise ValueError("object has no fallback Connector")
    return node


def validate_coordinates(coordinates):
    """Return coordinates as floats after checking shape and finiteness."""
    if isinstance(coordinates, (str, bytes)):
        raise ValueError("coordinates must contain three finite numbers")
    try:
        values = [float(value) for value in coordinates]
    except (TypeError, ValueError):
        raise ValueError("coordinates must contain three finite numbers") from None
    if len(values) != 3 or not all(isfinite(value) for value in values):
        raise ValueError("coordinates must contain three finite numbers")
    return values


def teleport_node(node, coordinates):
    """Set a node's translation and clear its velocity."""
    translation = node.getField("translation")
    if translation is None:
        raise ValueError("node has no translation field")
    translation.setSFVec3f(validate_coordinates(coordinates))
    node.resetPhysics()


def set_yaw(node, yaw):
    """Set a top-level node's heading while keeping it upright."""
    rotation = node.getField("rotation")
    if rotation is None:
        raise ValueError("node has no rotation field")
    rotation.setSFRotation([0.0, 0.0, 1.0, float(yaw)])


def _positive_number(node, field_name):
    field = node.getField(field_name)
    if field is None or field.getTypeName() not in ("SFFloat", "SFInt32"):
        return None
    value = float(field.getSFFloat() if field.getTypeName() == "SFFloat" else field.getSFInt32())
    return value if value > 0.0 else None


def _node_yaw(node):
    rotation = node.getField("rotation")
    if rotation is None:
        return 0.0
    x, y, z, angle = rotation.getSFRotation()
    return angle if abs(z) >= abs(x) and abs(z) >= abs(y) else 0.0


def _obstacle_shape(node):
    """Estimate a top-level Solid's planar footprint from exposed fields."""
    size = node.getField("size")
    if size is not None and size.getTypeName() == "SFVec3f":
        x, y, _ = size.getSFVec3f()
        return "box", x / 2.0, y / 2.0, _node_yaw(node)

    radius = _positive_number(node, "radius")
    if radius is not None:
        return "circle", radius

    width = _positive_number(node, "width")
    depth = _positive_number(node, "depth")
    if width is not None or depth is not None:
        return (
            "box",
            (width or DEFAULT_OBSTACLE_RADIUS) / 2.0,
            (depth or DEFAULT_OBSTACLE_RADIUS) / 2.0,
            _node_yaw(node),
        )
    return "circle", DEFAULT_OBSTACLE_RADIUS


def _distance_to_shape(x, y, position, shape):
    if shape[0] == "circle":
        return hypot(x - position[0], y - position[1]) - shape[1]

    _, half_x, half_y, yaw = shape
    delta_x = x - position[0]
    delta_y = y - position[1]
    local_x = delta_x * cos(yaw) + delta_y * sin(yaw)
    local_y = -delta_x * sin(yaw) + delta_y * cos(yaw)
    outside_x = max(abs(local_x) - half_x, 0.0)
    outside_y = max(abs(local_y) - half_y, 0.0)
    return hypot(outside_x, outside_y)


def _top_level_solids(supervisor):
    children = supervisor.getRoot().getField("children")
    for index in range(children.getCount()):
        node = children.getMFNode(index)
        if node.getBaseTypeName() in ("Solid", "Robot"):
            yield node


def find_clear_pose(supervisor, robot, target, distance, clearance, samples=24):
    """Return the nearest sampled stand-off pose clear of top-level solids."""
    distance = float(distance)
    clearance = float(clearance)
    if not isfinite(distance) or distance <= ROBOT_RADIUS:
        raise ValueError(f"distance must be greater than {ROBOT_RADIUS}")
    if not isfinite(clearance) or clearance < 0.0:
        raise ValueError("clearance must be a non-negative finite number")

    robot_position = get_world_position(robot)
    target_position = get_world_position(target)
    ignored_ids = {robot.getId(), target.getId()}
    obstacles = []
    for node in _top_level_solids(supervisor):
        if node.getId() not in ignored_ids:
            obstacles.append((get_world_position(node), _obstacle_shape(node)))

    start_angle = atan2(robot_position[1] - target_position[1], robot_position[0] - target_position[0])
    candidates = []
    for index in range(samples):
        angle = start_angle + 2.0 * pi * index / samples
        x = target_position[0] + distance * cos(angle)
        y = target_position[1] + distance * sin(angle)
        if all(_distance_to_shape(x, y, position, shape) >= ROBOT_RADIUS + clearance
               for position, shape in obstacles):
            travel = hypot(x - robot_position[0], y - robot_position[1])
            yaw = atan2(target_position[1] - y, target_position[0] - x)
            candidates.append((travel, [x, y, robot_position[2]], yaw))

    if not candidates:
        raise ValueError("no clear pose found near object")
    _, coordinates, yaw = min(candidates, key=lambda candidate: candidate[0])
    return coordinates, yaw


def set_hinge_position(node, position):
    """Set a HingeJoint, accepting either the joint or its named endpoint."""
    joint_parameters = node.getField("jointParameters")
    if joint_parameters is None:
        node = node.getParentNode()
        joint_parameters = node and node.getField("jointParameters")
    if joint_parameters is None:
        raise ValueError("object is not the endpoint of a HingeJoint")
    parameters_node = joint_parameters.getSFNode()
    position_field = parameters_node and parameters_node.getField("position")
    if position_field is None:
        raise ValueError("HingeJoint has no position field")
    position_field.setSFFloat(float(position))
