"""
Contents
--------
walk_nodes()           - Walks a node and its exposed descendants.
get_node()             - Retrieves a Webots node by its name field.
get_descendant()       - Retrieves a named node below another node.
get_slot_connector()   - Retrieves a Connector inside a robot slot PROTO.
get_object_connector() - Retrieves a Connector inside an object PROTO.
validate_coordinates() - Validates an x, y, z coordinate sequence.
teleport_node()        - Moves a node directly to world coordinates.
set_hinge_position()   - Sets a HingeJoint position in radians.
"""

from math import isfinite


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
