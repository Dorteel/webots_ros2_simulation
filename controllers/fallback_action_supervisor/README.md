# Fallback action supervisor

One Webots `Supervisor` performs fallback actions for every robot. `fallback_action_supervisor.py`
owns the simulation loop, `command_server.py` accepts local commands, `actions.py` defines action
semantics, and `world_utils.py` contains robot-independent world operations. `action_cli.py` is the
command-line client.

## Webots setup

Add one supervisor robot to the world:

```webots
Robot {
  name "oracle"
  controller "fallback_action_supervisor"
  supervisor TRUE
}
```

Give every controlled robot and object a unique Webots `name`. Actions use these names directly;
DEF names are not required. Names are case-sensitive, so the apartment robot must be addressed as
`TIAGo++`. One `oracle` Supervisor controls every named robot in the world.

`pick` uses compatible Webots Connectors in the selected robot's `endEffectorRightSlot` and in the
object. The Connector snaps the object to the gripper and maintains a physical link; no per-step
teleport is used. `place` unlocks the Connector before teleporting the object. Each robot can hold
one object, and picking another object releases its previous one.

The included `TiagoGripperConnector.proto` provides the only active connector. The local object
PROTOs provide passive connectors for all loose food, drink, utensil, container, book, computer,
telephone, toy, cookware, and fire-extinguisher instances in `complete_apartment_tiago.wbt`.
Furniture, fixtures, plants, paintings, appliances, people, and robots are intentionally excluded.
New pickable object types need a local PROTO containing
`DEF FALLBACK_OBJECT_CONNECTOR Connector` and a public `connectorModel` field bound to the
Connector's `model`. The gripper similarly exposes `connectorModel` and `connectorLocked`, allowing
the Supervisor to control it without writing read-only internal PROTO fields.

Implemented actions match `actions.md`: `move`, `move_to_object`, `pick`, `place`, `open`, and `close`. Coordinates
are `[x, y, z]` in world coordinates. Open and close target the unique name of a HingeJoint's
endpoint object.

## Command-line examples

Start `complete_apartment_tiago.wbt`, run the simulation, and issue commands from the repository
root. Replace the example object names with unique names present in your world.

### Move

Teleport `TIAGo++` to world coordinates `[-1, -2, 0.095]`:

```bash
python3 controllers/fallback_action_supervisor/action_cli.py move 'TIAGo++' -1 -2 0.095
```

### Pick

Snap and connect `plate(9)` to TIAGo's right end effector. Parentheses have a special meaning in
Bash, so quote the object name:

```bash
python3 controllers/fallback_action_supervisor/action_cli.py pick 'TIAGo++' 'plate(9)'
```

### Move to object

Move to the nearest sampled clear pose, starting 0.8 m from `plate(9)`, and face it:

```bash
python3 controllers/fallback_action_supervisor/action_cli.py move_to_object 'TIAGo++' 'plate(9)'
```

Override the stand-off distance and extra obstacle clearance when needed:

```bash
python3 controllers/fallback_action_supervisor/action_cli.py move_to_object 'TIAGo++' 'plate(9)' --distance 1.0 --clearance 0.2
```

This action checks sampled poses against conservative planar footprints of top-level Webots
solids. If the requested distance is blocked, it searches outward in 0.1 m steps up to 1.2 m
farther. It returns an error without moving when none are clear; it is a fallback placement check,
not a replacement for a navigation planner.

### Place

Disconnect `plate(9)` and teleport it to world coordinates `[-1, -1, 0.5]`:

```bash
python3 controllers/fallback_action_supervisor/action_cli.py place 'TIAGo++' 'plate(9)' -1 -1 0.5
```

### Open

Set the hinge containing the named endpoint `fridge_door` to 80 degrees:

```bash
python3 controllers/fallback_action_supervisor/action_cli.py open 'TIAGo++' fridge_door
```

### Close

Set the same hinge to 0 degrees:

```bash
python3 controllers/fallback_action_supervisor/action_cli.py close 'TIAGo++' fridge_door
```

The server listens only on `127.0.0.1:8765`. A successful command prints `ok`; validation and name
lookup failures are returned to the terminal.

Future ROS1 or ROS2 action servers should reuse this controller and call:

```python
execute_action(supervisor, "move", {"robot": "ROBOT_A", "coordinates": [1, 2, 0]})
```

The ROS adapter should translate its goal into this action name and plain parameter dictionary;
the implementations in `actions.py` remain ROS-independent.
