"""Nonblocking look/turn actions, advanced only by the Supervisor thread.

One goal reserves all five interfaces. No executor thread calls Webots APIs.
Turns conservatively require a graph without Nav2/exploration and a sole velocity
publisher; stop the navigation launch and use basic simulation for these actions.
"""
from functools import partial
import math
import time
from pathlib import Path

import yaml
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.qos import qos_profile_sensor_data
from rclpy.task import Future
from simulation_actions.action import Look, LookAhead, Turn

from head_gaze import PAN_JOINT, TILT_JOINT, head_joints
from world_utils import get_node


def requested_angle(value, default):
    """Zero selects default; direction belongs to the action name."""
    if not math.isfinite(value) or value < 0:
        raise ValueError('angle must be finite and nonnegative; zero selects the default')
    return default if value == 0 else value


def yaw(message):
    q = message.pose.pose.orientation
    values = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values) or sum(v*v for v in values) < 1e-12:
        raise ValueError('Invalid odometry orientation')
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))


def angle_delta(current, previous):
    return math.atan2(math.sin(current-previous), math.cos(current-previous))


def load_settings(path):
    settings = yaml.safe_load(Path(path).read_text())
    for section, keys in {
        'look': ('default_angle', 'tolerance', 'timeout'),
        'turn': ('default_angle', 'angular_velocity', 'tolerance', 'timeout',
                 'odom_timeout', 'stall_timeout'),
    }.items():
        for key in keys:
            value = settings[section][key]
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{section}.{key} must be finite and positive')
    for key in ('neutral_pan', 'neutral_tilt'):
        if not math.isfinite(settings['look'][key]):
            raise ValueError(f'look.{key} must be finite')
    return settings


class LookTurnActions:
    """Action callbacks suspend on Futures; step() performs bounded work."""

    TYPES = {'look_left': Look, 'look_right': Look, 'look_ahead': LookAhead,
             'turn_left': Turn, 'turn_right': Turn}
    NAVIGATION_NODES = {'bt_navigator', 'controller_server', 'behavior_server',
                        'velocity_smoother', 'collision_monitor', 'explore_node'}

    def __init__(self, node, supervisor, config_path):
        self.node, self.supervisor = node, supervisor
        self.settings = load_settings(config_path)
        self.reserved = False
        self.active = None
        self.publisher = None
        self.command_type = None
        self.odometry = None
        self.odom_received = 0.0
        self.subscription = node.create_subscription(
            Odometry, '/odom', self.receive_odom, qos_profile_sensor_data)
        # Also guard against navigation goals while a turn is running. Graph
        # checks below deliberately reject even idle Nav2 nodes.
        self.navigation_active = set()
        self.status_subscriptions = [node.create_subscription(
            GoalStatusArray,
            f'/{action}/_action/status',
            lambda msg, action=action: self.navigation_status(action, msg), 10)
            for action in ('navigate_to_pose', 'navigate_through_poses', 'follow_path', 'spin')]
        self.servers = [ActionServer(
            node, kind, '/' + name,
            goal_callback=lambda request, name=name: self.accept(name, request),
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            execute_callback=partial(self.execute, name),
        ) for name, kind in self.TYPES.items()]

    def receive_odom(self, message):
        self.odometry = message
        self.odom_received = time.monotonic()

    def navigation_status(self, action, message):
        busy = any(s.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
                               GoalStatus.STATUS_CANCELING) for s in message.status_list)
        if busy:
            self.navigation_active.add(action)
        else:
            self.navigation_active.discard(action)

    def navigation_present(self):
        return bool(self.navigation_active or
                    self.NAVIGATION_NODES.intersection(self.node.get_node_names()))

    def accept(self, name, request):
        try:
            if name != 'look_ahead':
                requested_angle(request.angle, self.settings['turn' if name.startswith('turn') else 'look']['default_angle'])
            if self.reserved:
                raise ValueError('Another look/turn action is already running')
            if name.startswith('turn') and self.navigation_present():
                raise ValueError('Turns require basic simulation: stop Nav2/exploration first')
        except ValueError as error:
            self.node.get_logger().warning(str(error))
            return GoalResponse.REJECT
        self.reserved = True
        return GoalResponse.ACCEPT

    async def execute(self, name, goal):
        future = Future()
        state = dict(name=name, goal=goal, future=future, started=time.monotonic(),
                     achieved=0.0, initialized=False)
        self.active = state
        try:
            return await future
        finally:
            if self.active is state:
                self.active = None
            self.reserved = False

    def finish(self, success, message, canceled=False):
        state = self.active
        if state is None or state['future'].done():
            return
        if state['name'].startswith('turn'):
            self.stop()
        kind = self.TYPES[state['name']]
        result = kind.Result(success=success, message=message)
        if state['name'] != 'look_ahead':
            result.achieved_angle = abs(state['achieved'])
        if self.node.context.ok():
            if canceled:
                state['goal'].canceled()
            elif success:
                state['goal'].succeed()
            else:
                state['goal'].abort()
        state['future'].set_result(result)

    def initialize_head(self, state):
        robot = get_node(self.supervisor, 'TIAGo', 'robot')
        joints = head_joints(robot)
        config = self.settings['look']
        pan = joints[PAN_JOINT][1].getSFFloat()
        state.update(joints=joints, initial_pan=pan)
        if state['name'] == 'look_ahead':
            targets = {PAN_JOINT: config['neutral_pan'], TILT_JOINT: config['neutral_tilt']}
        else:
            direction = 1 if state['name'] == 'look_left' else -1
            amount = requested_angle(state['goal'].request.angle, config['default_angle'])
            targets = {PAN_JOINT: pan + direction*amount}
        for name, target in targets.items():
            joint, field, minimum, maximum = joints[name]
            if not all(math.isfinite(v) for v in (field.getSFFloat(), minimum, maximum, target)) or minimum >= maximum:
                raise ValueError(f'Invalid head joint state/limits: {name}')
            if state['name'] == 'look_ahead' and not minimum <= target <= maximum:
                raise ValueError(f'Configured neutral position is outside limits: {name}')
            targets[name] = max(minimum, min(maximum, target))
        state['targets'] = targets
        # Supervisor joint writes hold the motor target. Do not step recursively;
        # read back only after the outer simulation loop advances.
        for name, target in targets.items():
            joints[name][0].setJointPosition(target, 1)

    def velocity_interface(self):
        subscriptions = self.node.get_subscriptions_info_by_topic('/cmd_vel')
        types = {s.topic_type for s in subscriptions if s.node_name == 'diffdrive_controller'}
        supported = {'geometry_msgs/msg/Twist': Twist,
                     'geometry_msgs/msg/TwistStamped': TwistStamped}
        if len(types) != 1 or next(iter(types)) not in supported:
            raise ValueError('No unambiguous diffdrive_controller /cmd_vel interface available')
        kind = supported[next(iter(types))]
        if self.publisher is not None and self.command_type is not kind:
            self.node.destroy_publisher(self.publisher)
            self.publisher = None
        self.command_type = kind
        if self.publisher is None:
            self.publisher = self.node.create_publisher(kind, '/cmd_vel', 10)

    def send(self, speed):
        if self.publisher is None:
            return
        msg = self.command_type()
        twist = msg.twist if self.command_type is TwistStamped else msg
        if self.command_type is TwistStamped:
            msg.header.stamp = self.odometry.header.stamp
            msg.header.frame_id = 'base_link'
        twist.angular.z = speed
        self.publisher.publish(msg)

    def stop(self):
        # Repeated zeros improve delivery during cancellation and orderly shutdown.
        if not self.node.context.ok():
            return  # DDS is already shut down; wheel command timeout is the fallback.
        for _ in range(3):
            try:
                self.send(0.0)
            except Exception as error:
                self.node.get_logger().warning(f'Could not publish stop: {error}')
                break

    def step(self):
        state = self.active
        if state is None or state['future'].done():
            return
        try:
            turning = state['name'].startswith('turn')
            config = self.settings['turn' if turning else 'look']
            now = time.monotonic()
            if state['goal'].is_cancel_requested:
                self.finish(False, 'Canceled', canceled=True)
                return
            if now - state['started'] > config['timeout']:
                raise ValueError('Action timed out')
            if turning:
                self.step_turn(state, config, now)
            elif not state['initialized']:
                self.initialize_head(state)
                state['initialized'] = True
            else:
                pan = state['joints'][PAN_JOINT][1].getSFFloat()
                readings = {name: state['joints'][name][1].getSFFloat()
                            for name in state['targets']}
                if not all(math.isfinite(v) for v in (pan, *readings.values())):
                    raise ValueError('Nonfinite head joint readback')
                state['achieved'] = pan - state['initial_pan']
                remaining = max(abs(readings[name]-target)
                                for name, target in state['targets'].items())
                state['goal'].publish_feedback(self.TYPES[state['name']].Feedback(remaining_angle=remaining))
                if remaining <= config['tolerance']:
                    self.finish(True, 'Head target reached (clamped to mechanical limits where necessary)')
        except Exception as error:
            self.finish(False, str(error))

    def step_turn(self, state, config, now):
        if self.navigation_present():
            raise ValueError('Nav2/exploration detected; turn stopped')
        if self.odometry is None or now-self.odom_received > config['odom_timeout']:
            raise ValueError('Ground-truth odometry is unavailable or stale')
        current = yaw(self.odometry)
        if not state['initialized']:
            self.velocity_interface()
            state.update(initialized=True, previous=current, progress_time=now, progress_angle=0.0,
                         target=requested_angle(state['goal'].request.angle, config['default_angle']))
        else:
            state['achieved'] += angle_delta(current, state['previous'])
            state['previous'] = current
        if self.publisher.get_subscription_count() == 0:
            if now-state['started'] < config['odom_timeout']:
                return  # Allow DDS endpoint discovery after publisher creation.
            raise ValueError('Wheel command subscriber is unavailable')
        if len(self.node.get_publishers_info_by_topic('/cmd_vel')) > 1:
            raise ValueError('Competing /cmd_vel publisher detected; turn stopped')
        direction = 1 if state['name'] == 'turn_left' else -1
        remaining = state['target'] - direction*state['achieved']
        state['goal'].publish_feedback(Turn.Feedback(remaining_angle=abs(remaining)))
        speed = self.odometry.twist.twist.angular.z
        if not math.isfinite(speed):
            raise ValueError('Invalid odometry velocity')
        if abs(remaining) <= config['tolerance']:
            self.send(0.0)
            if abs(speed) < 0.03:
                self.finish(True, 'Base rotation reached and stopped')
            return
        if abs(state['achieved']-state['progress_angle']) >= 0.01:
            state.update(progress_time=now, progress_angle=state['achieved'])
        if now-state['progress_time'] > config['stall_timeout']:
            raise ValueError('Base rotation stalled')
        # Proportional slowdown with a small minimum outside the tolerance;
        # signed error also corrects overshoot and supports rotations beyond pi.
        magnitude = min(config['angular_velocity'], max(0.05, abs(remaining)))
        self.send(direction * math.copysign(magnitude, remaining))

    def destroy(self):
        if self.active is not None and not self.active['future'].done():
            self.finish(False, 'Supervisor shutting down')
        self.stop()
        for server in self.servers:
            server.destroy()
        self.node.destroy_subscription(self.subscription)
        for subscription in self.status_subscriptions:
            self.node.destroy_subscription(subscription)
        if self.publisher is not None:
            self.node.destroy_publisher(self.publisher)
