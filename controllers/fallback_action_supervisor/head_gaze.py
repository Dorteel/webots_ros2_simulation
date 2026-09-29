"""Demo head-only gaze using the existing Supervisor joint-position mechanism."""
from math import atan2, hypot, isfinite, radians

if __package__:
    from .world_utils import get_node, validate_coordinates
else:
    from world_utils import get_node, validate_coordinates

LOOK_PAN_STEP_DEG = 30
PAN_JOINT = "head_1_joint"
TILT_JOINT = "head_2_joint"


def head_joints(robot):
    pending, found = [robot], {}
    while pending:
        node = pending.pop()
        # Internal PROTO nodes have no public ID (getId returns -1 and logs
        # an error). Walk the expanded scene tree without ID-based deduplication.
        # Visit the expanded fields once, rather than both the PROTO interface
        # and its expansion (which can expose the same devices twice).
        if node.isProto():
            fields = [node.getBaseNodeFieldByIndex(i)
                      for i in range(node.getNumberOfBaseNodeFields())]
        else:
            fields = [node.getFieldByIndex(i) for i in range(node.getNumberOfFields())]
        if node.getTypeName() == 'HingeJoint':
            devices = node.getField('device')
            for i in range(devices.getCount() if devices else 0):
                motor = devices.getMFNode(i)
                name = motor.getField('name')
                if name and name.getSFString() in (PAN_JOINT, TILT_JOINT):
                    key = name.getSFString()
                    if key in found:
                        raise ValueError(f'Ambiguous head joint {key}')
                    params = node.getField('jointParameters').getSFNode()
                    found[key] = (node, params.getField('position'),
                                  motor.getField('minPosition').getSFFloat(),
                                  motor.getField('maxPosition').getSFFloat())
        for field in fields:
            if field.getTypeName() == 'SFNode':
                child = field.getSFNode()
                if child is not None:
                    pending.append(child)
            elif field.getTypeName() == 'MFNode':
                pending.extend(field.getMFNode(i) for i in range(field.getCount()))
    for role, name in (('pan', PAN_JOINT), ('tilt', TILT_JOINT)):
        if name not in found:
            raise ValueError(
                f"Head {role} joint '{name}' not present in TIAGo's expanded Webots "
                "HingeJoint/RotationalMotor interface")
    return found


def gaze(supervisor, action, optical_target=None):
    if action not in ('look-at', 'look-left', 'look-right'):
        raise ValueError('Unsupported gaze action')
    robot = get_node(supervisor, 'TIAGo', 'robot')
    joints = head_joints(robot)
    print('[SEARCH] Head joints:', flush=True)
    print(f'[SEARCH]   pan={PAN_JOINT}', flush=True)
    print(f'[SEARCH]   tilt={TILT_JOINT}', flush=True)
    print(f'[SEARCH]   current pan={joints[PAN_JOINT][1].getSFFloat():.6f} rad', flush=True)
    print(f'[SEARCH]   current tilt={joints[TILT_JOINT][1].getSFFloat():.6f} rad', flush=True)
    if action == 'look-at':
        x, y, z = validate_coordinates(optical_target)
        if z <= 0:
            raise ValueError('Gaze target is behind the optical camera')
        # ROS optical +X right, +Y down, +Z forward. TIAGo tilt + is up.
        changes = {PAN_JOINT: -atan2(x, z), TILT_JOINT: -atan2(y, hypot(x, z))}
    else:
        changes = {PAN_JOINT: radians(LOOK_PAN_STEP_DEG) * (1 if action == 'look-left' else -1)}
    targets = {}
    for name, change in changes.items():
        joint, field, minimum, maximum = joints[name]
        current = field.getSFFloat()
        if not all(isfinite(v) for v in (current, minimum, maximum)) or minimum >= maximum:
            raise ValueError(f'Invalid physical limits for {name}')
        targets[name] = max(minimum, min(maximum, current + change))
    if action != 'look-at' and abs(targets[PAN_JOINT] - joints[PAN_JOINT][1].getSFFloat()) < 1e-5:
        raise ValueError('Head pan limit reached; gaze did not move')
    for name, target in targets.items():
        joint, field, _, _ = joints[name]
        # Webots also updates the motor target, holding this pose for Sense.
        joint.setJointPosition(target, 1)
    # Supervisor writes are queued until the next simulation step.
    if supervisor.step(int(supervisor.getBasicTimeStep())) == -1:
        raise ValueError('Webots stopped before head gaze could be applied')
    for name, target in targets.items():
        field = joints[name][1]
        if abs(field.getSFFloat() - target) > 1e-4:
            raise ValueError(f'Head joint did not reach target: {name}')
    return {'head_positions': targets}
