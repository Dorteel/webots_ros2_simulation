"""Mapping-only runtime door preparation; never saves or edits the world file."""
import json
import math

from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String

from world_utils import get_node


def door_hinge(door):
    """Door's own hinge is a direct generated child, not a handle hinge."""
    if door.getTypeName() != 'Door' or not door.getField('canBeOpen').getSFBool():
        raise ValueError('expected an openable Door PROTO')
    if door.getField('selfClosing').getSFBool():
        raise ValueError('self-closing doors are not supported by mapping preparation')
    children = door.getBaseNodeField('children')
    hinges = [children.getMFNode(i) for i in range(children.getCount())
              if children.getMFNode(i).getTypeName() == 'HingeJoint']
    if len(hinges) != 1:
        raise ValueError('Door must have exactly one direct hinge')
    return hinges[0]


class MappingDoors:
    def __init__(self, supervisor, node, config_path, status_topic):
        self.supervisor, self.node = supervisor, node
        self.publisher = node.create_publisher(
            String, status_topic,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.done = False
        self.stable_since = None
        self.started = supervisor.getTime()
        self.doors = []
        self.publish('opening')
        try:
            with open(config_path, encoding='utf-8') as stream:
                config = json.load(stream)
            names = config['doors']
            if not names or len(set(names)) != len(names):
                raise ValueError('door names must be nonempty and unique')
            angle = float(config['open_angle'])
            self.tolerance = float(config['angle_tolerance'])
            self.settle = float(config['settle_seconds'])
            self.timeout = float(config['timeout_seconds'])
            if not all(math.isfinite(v) and v > 0 for v in
                       (angle, self.tolerance, self.settle, self.timeout)):
                raise ValueError('door settings must be finite and positive')
            if self.settle >= self.timeout or self.tolerance >= angle / 2:
                raise ValueError('invalid settling timeout or angle tolerance')
            # Validate every configured door before changing any of them.
            for name in names:
                door = get_node(supervisor, name, 'mapping door')
                hinge = door_hinge(door)
                params = hinge.getField('jointParameters').getSFNode()
                target = -angle if door.getField('jointAtLeft').getSFBool() else angle
                if not params.getField('minStop').getSFFloat() <= target <= params.getField('maxStop').getSFFloat():
                    raise ValueError(f'{name}: open angle is outside hinge limits')
                leaf = hinge.getField('endPoint').getSFNode()
                self.doors.append((name, hinge, params.getField('position'), leaf, target))
            for name, hinge, _, leaf, target in self.doors:
                hinge.setJointPosition(target, 1)
                leaf.resetPhysics()
                node.get_logger().info(f'Mapping door opened: {name} ({target:.2f} rad)')
        except Exception as error:
            self.fail(str(error))

    def publish(self, state):
        self.publisher.publish(String(data=state))

    def fail(self, reason):
        self.done = True
        self.node.get_logger().error(f'Mapping door preparation failed: {reason}')
        self.publish(f'error: {reason}')

    def step(self):
        if self.done:
            return
        try:
            now = self.supervisor.getTime()
            unsettled = []
            for name, _, position, leaf, target in self.doors:
                velocity = leaf.getVelocity()
                if (abs(position.getSFFloat() - target) > self.tolerance or
                        any(abs(v) > 0.02 for v in velocity[:3]) or
                        any(abs(v) > 0.05 for v in velocity[3:])):
                    unsettled.append(name)
            if unsettled:
                self.stable_since = None
            elif self.stable_since is None:
                self.stable_since = now
            elif now - self.stable_since >= self.settle:
                self.done = True
                self.node.get_logger().info(
                    f'All {len(self.doors)} mapping doors open and settled; mapping ready')
                self.publish('ready: ' + json.dumps({
                    name: round(position.getSFFloat(), 4)
                    for name, _, position, _, _ in self.doors}))
                return
            if now - self.started >= self.timeout:
                self.fail('doors did not settle: ' + ', '.join(unsettled or ['settling interval incomplete']))
        except Exception as error:
            self.fail(str(error))
