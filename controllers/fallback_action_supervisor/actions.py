"""
Contents
--------
execute_action() - Validates and dispatches a fallback action.
release_pick()    - Unlocks a robot's current Connector attachment.
move()           - Teleports a robot to world coordinates.
pick()           - Connects an object to a robot's end effector.
place()          - Teleports an object to world coordinates.
open_object()    - Opens a HingeJoint to 80 degrees.
close_object()   - Closes a HingeJoint to 0 degrees.
"""

from math import radians

from world_utils import (
    get_node,
    get_object_connector,
    get_slot_connector,
    set_hinge_position,
    teleport_node,
)


_CONNECTIONS = {}


def release_pick(robot):
    """Unlock and disable a robot's current Connector pair."""
    connection = _CONNECTIONS.pop(robot, None)
    if connection is None:
        return
    gripper_connector, object_connector, _ = connection
    gripper_connector.getField("connectorLocked").setSFBool(False)
    gripper_connector.getField("connectorModel").setSFString("fallback disabled")
    object_connector.getField("connectorModel").setSFString("fallback disabled")


def move(supervisor, robot, coordinates):
    """Teleport the robot identified by its name field."""
    teleport_node(get_node(supervisor, robot, "robot"), coordinates)


def pick(supervisor, robot, object):
    """Snap and lock an object's passive Connector to the right gripper."""
    robot_node = get_node(supervisor, robot, "robot")
    item = get_node(supervisor, object, "object")
    gripper_connector = get_slot_connector(robot_node, "endEffectorRightSlot")
    object_connector = get_object_connector(item)

    for held_by, connection in list(_CONNECTIONS.items()):
        if held_by == robot or connection[2].getId() == item.getId():
            release_pick(held_by)

    model = f"fallback grasp {item.getId()}"
    gripper_connector.getField("connectorModel").setSFString(model)
    object_connector.getField("connectorModel").setSFString(model)
    gripper_connector.getField("connectorLocked").setSFBool(True)
    _CONNECTIONS[robot] = (gripper_connector, object_connector, item)
    return {"connector_model": model}


def place(supervisor, robot, object, coordinates):
    """Teleport an object to world coordinates."""
    get_node(supervisor, robot, "robot")
    item = get_node(supervisor, object, "object")
    connection = _CONNECTIONS.get(robot)
    if connection is not None and connection[2].getId() == item.getId():
        release_pick(robot)
    teleport_node(item, coordinates)


def open_object(supervisor, robot, object):
    """Set a HingeJoint to 80 degrees."""
    get_node(supervisor, robot, "robot")
    set_hinge_position(get_node(supervisor, object, "object"), radians(80))


def close_object(supervisor, robot, object):
    """Set a HingeJoint to 0 degrees."""
    get_node(supervisor, robot, "robot")
    set_hinge_position(get_node(supervisor, object, "object"), 0.0)


_ACTIONS = {
    "move": (move, ("robot", "coordinates")),
    "pick": (pick, ("robot", "object")),
    "place": (place, ("robot", "object", "coordinates")),
    "open": (open_object, ("robot", "object")),
    "close": (close_object, ("robot", "object")),
}


def execute_action(supervisor, action, parameters):
    """Execute an action from a plain dictionary, suitable for ROS adapters."""
    if not isinstance(action, str) or action not in _ACTIONS:
        raise ValueError(f"unsupported action: {action!r}")
    if not isinstance(parameters, dict):
        raise ValueError("parameters must be a dictionary")

    function, required = _ACTIONS[action]
    missing = [field for field in required if field not in parameters]
    if missing:
        raise ValueError(f"missing required fields: {', '.join(missing)}")
    unexpected = [field for field in parameters if field not in required]
    if unexpected:
        raise ValueError(f"unexpected fields: {', '.join(unexpected)}")
    return function(supervisor, **parameters)
