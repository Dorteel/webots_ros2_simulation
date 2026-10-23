"""State-machine and real ROS action tests with deterministic simulated hardware."""
import math
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'controllers/fallback_action_supervisor'))
import look_turn_actions as module
from look_turn_actions import LookTurnActions, angle_delta, requested_angle
from rclpy.action import ActionClient, GoalResponse
from rclpy.executors import SingleThreadedExecutor
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistStamped
from simulation_actions.action import Look, LookAhead, Turn


class Field:
    def __init__(self, value):
        self.value = value

    def getSFFloat(self):
        return self.value


class Joint:
    def __init__(self, field):
        self.field = field
        self.commands = []

    def setJointPosition(self, value, axis):
        self.commands.append(value)
        self.field.value = value


class Goal:
    def __init__(self, angle=0.0):
        self.request = Look.Goal(angle=angle)
        self.is_cancel_requested = False
        self.feedback = []
        self.status = None

    def publish_feedback(self, feedback):
        self.feedback.append(feedback.remaining_angle)

    def succeed(self):
        self.status = 'succeeded'

    def abort(self):
        self.status = 'aborted'

    def canceled(self):
        self.status = 'canceled'


class LookTurnTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('look_turn_test_server')
        self.supervisor = object()
        self.actions = LookTurnActions(self.node, self.supervisor, ROOT / 'config/look_turn.yaml')
        self.pan, self.tilt = Field(0.0), Field(0.0)
        self.joints = {
            module.PAN_JOINT: (Joint(self.pan), self.pan, -1.2, 1.2),
            module.TILT_JOINT: (Joint(self.tilt), self.tilt, -0.6, 0.6)}
        self.patches = [patch.object(module, 'get_node', return_value=object()),
                        patch.object(module, 'head_joints', return_value=self.joints)]
        for item in self.patches:
            item.start()

    def tearDown(self):
        self.actions.destroy()
        self.node.destroy_node()
        for item in self.patches:
            item.stop()

    def begin(self, name, angle=0.0):
        goal = Goal(angle)
        future = module.Future()
        self.actions.reserved = True
        self.actions.active = dict(name=name, goal=goal, future=future,
                                   started=time.monotonic(), initialized=False, achieved=0.0)
        return goal, future

    def odom(self, angle=0.0, speed=0.0):
        msg = Odometry()
        msg.pose.pose.orientation.z = math.sin(angle/2)
        msg.pose.pose.orientation.w = math.cos(angle/2)
        msg.twist.twist.angular.z = speed
        self.actions.receive_odom(msg)

    def test_defaults_validation_and_unwrapped_delta(self):
        self.assertEqual(requested_angle(0.0, 0.7), 0.7)
        self.assertEqual(requested_angle(0.8, 0.7), 0.8)
        for value in (-0.1, math.nan, math.inf, -math.inf):
            with self.assertRaises(ValueError):
                requested_angle(value, 0.7)
            self.assertEqual(self.actions.accept('look_left', Look.Goal(angle=value)), GoalResponse.REJECT)
        self.assertAlmostEqual(angle_delta(-3.1, 3.1), 2*math.pi-6.2)

    def test_head_defaults_custom_signs_neutral_and_limits(self):
        # No base interface is even created by a head action.
        for name, amount, expected in [('look_left', 0.0, math.pi/6),
                                       ('look_right', 0.2, math.pi/6-0.2),
                                       ('look_left', 9.0, 1.2)]:
            goal, future = self.begin(name, amount)
            self.actions.step()
            self.assertFalse(future.done())  # Readback waits for an outer step.
            self.actions.step()
            self.assertTrue(future.result().success)
            self.assertAlmostEqual(self.pan.value, expected)
            self.assertTrue(goal.feedback)
            self.assertIsNone(self.actions.publisher)
        self.tilt.value = -0.4
        goal, future = self.begin('look_ahead')
        self.actions.step()
        self.actions.step()
        self.assertTrue(future.result().success)
        self.assertEqual((self.pan.value, self.tilt.value), (0.0, 0.0))

    def test_head_unavailable_cancellation_timeout_and_conflict(self):
        self.assertEqual(self.actions.accept('look_left', Look.Goal()), GoalResponse.ACCEPT)
        self.assertEqual(self.actions.accept('turn_left', Turn.Goal()), GoalResponse.REJECT)
        goal, future = self.begin('look_left')
        goal.is_cancel_requested = True
        self.actions.step()
        self.assertEqual(goal.status, 'canceled')
        self.assertFalse(future.result().success)
        goal, future = self.begin('look_right')
        self.actions.active['started'] -= 100
        self.actions.step()
        self.assertIn('timed out', future.result().message)
        with patch.object(module, 'head_joints', side_effect=ValueError('Head unavailable')):
            goal, future = self.begin('look_left')
            self.actions.step()
            self.assertIn('unavailable', future.result().message)

    def setup_turn(self, name='turn_left', angle=0.0):
        goal, future = self.begin(name, angle)
        self.odom()
        # Stub only DDS discovery/transport; actual control algorithm is tested.
        self.sent = []
        self.actions.velocity_interface = lambda: None
        self.actions.publisher = unittest.mock.Mock()
        self.actions.publisher.get_subscription_count.return_value = 1
        self.actions.send = self.sent.append
        self.actions.navigation_present = lambda: False
        self.actions.node.get_publishers_info_by_topic = lambda _: [object()]
        self.actions.step()
        return goal, future

    def test_turn_defaults_signs_multiturn_and_head_isolation(self):
        for name, angle, sign in [('turn_left', 0.0, 1), ('turn_right', 0.7, -1),
                                  ('turn_left', 7.0, 1)]:
            goal, future = self.setup_turn(name, angle)
            target = angle or math.pi/2
            self.assertAlmostEqual(self.actions.active['target'], target)
            self.assertEqual(math.copysign(1, self.sent[-1]), sign)
            for index in range(1, 101):
                self.odom(sign*target*index/100)
                self.actions.step()
            self.assertTrue(future.result().success)
            self.assertAlmostEqual(future.result().achieved_angle, target, delta=0.03)
            self.assertEqual(self.sent[-1], 0.0)
            self.assertEqual((self.pan.value, self.tilt.value), (0, 0))
            self.assertFalse(self.joints[module.PAN_JOINT][0].commands)
            self.assertTrue(goal.feedback)
        self.actions.publisher = None  # Mock was not allocated by rclpy.

    def test_turn_stale_stall_cancel_timeout_navigation_and_competitor_stop(self):
        for failure in ('stale', 'stall', 'cancel', 'timeout', 'navigation', 'competitor'):
            goal, future = self.setup_turn()
            if failure == 'stale':
                self.actions.odom_received -= 10
            elif failure == 'stall':
                self.actions.active['progress_time'] -= 10
            elif failure == 'cancel':
                goal.is_cancel_requested = True
            elif failure == 'timeout':
                self.actions.active['started'] -= 100
            elif failure == 'navigation':
                self.actions.navigation_present = lambda: True
            else:
                self.actions.node.get_publishers_info_by_topic = lambda _: [object(), object()]
            self.actions.step()
            self.assertTrue(future.done(), failure)
            self.assertFalse(future.result().success, failure)
            self.assertEqual(self.sent[-1], 0.0, failure)
        self.actions.publisher = None

    def test_velocity_transport_types_and_unavailable_controller(self):
        from types import SimpleNamespace
        self.odom()
        self.actions.node.get_subscriptions_info_by_topic = lambda _: []
        with self.assertRaisesRegex(ValueError, 'diffdrive_controller'):
            self.actions.velocity_interface()
        for label, kind in [('Twist', Twist), ('TwistStamped', TwistStamped)]:
            self.actions.node.get_subscriptions_info_by_topic = lambda _, label=label: [
                SimpleNamespace(node_name='diffdrive_controller', topic_type='geometry_msgs/msg/'+label)]
            self.actions.velocity_interface()
            self.assertIs(self.actions.command_type, kind)
            # Inspect actual constructed ROS messages without a transport mock.
            messages = []
            real_publisher = self.actions.publisher
            self.actions.publisher = unittest.mock.Mock()
            self.actions.publisher.publish.side_effect = messages.append
            self.actions.send(-0.4)
            message = messages[-1]
            twist = message.twist if kind is TwistStamped else message
            self.assertEqual(twist.angular.z, -0.4)
            self.assertEqual(twist.linear.x, 0.0)
            if kind is TwistStamped:
                self.assertEqual(message.header.stamp, self.actions.odometry.header.stamp)
            self.actions.publisher = real_publisher

    def test_existing_seven_action_adapters(self):
        import pick_action_server
        import fallback_action_servers
        from simulation_actions.action import Pick
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)
        client_node = rclpy.create_node('legacy_action_test_client')
        executor.add_node(client_node)
        calls = []
        adapters = [pick_action_server.PickActionServer(self.node, self.supervisor),
                    fallback_action_servers.FallbackActionServers(self.node, self.supervisor)]
        types = {'pick': Pick, **fallback_action_servers.ACTION_TYPES}
        clients = {name: ActionClient(client_node, kind, '/'+name) for name, kind in types.items()}
        def wait(future):
            executor.spin_until_future_complete(future, timeout_sec=5)
            self.assertTrue(future.done())
            return future.result()
        try:
            with patch.object(pick_action_server, 'execute_action', side_effect=lambda *args: calls.append(args)), \
                 patch.object(fallback_action_servers, 'execute_action', side_effect=lambda *args: calls.append(args)):
                for name, client in clients.items():
                    self.assertTrue(client.wait_for_server(timeout_sec=5))
                    goal = types[name].Goal(robot='TIAGo', object='test_object')
                    handle = wait(client.send_goal_async(goal))
                    self.assertTrue(handle.accepted)
                    self.assertTrue(wait(handle.get_result_async()).result.success)
            self.assertEqual({args[1] for args in calls}, set(types))
        finally:
            for client in clients.values():
                client.destroy()
            for adapter in adapters:
                adapter.destroy()
            executor.remove_node(client_node)
            client_node.destroy_node()
            executor.remove_node(self.node)
            executor.shutdown()

    def test_real_ros_wheel_control_with_both_message_types(self):
        # A DDS-connected wheel stub integrates commands into odometry. This
        # exercises discovery, message transport, action result, and stop.
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)
        wheel = rclpy.create_node('diffdrive_controller')
        executor.add_node(wheel)
        odom_publisher = wheel.create_publisher(Odometry, '/odom', 10)
        client = ActionClient(wheel, Turn, '/turn_right')
        try:
            self.assertTrue(client.wait_for_server(timeout_sec=5))
            for kind in (Twist, TwistStamped):
                motion = {'yaw': 0.0, 'speed': 0.0, 'commands': []}
                def command(message):
                    value = message.twist if kind is TwistStamped else message
                    motion['speed'] = value.angular.z
                    motion['commands'].append(value.angular.z)
                subscription = wheel.create_subscription(kind, '/cmd_vel', command, 10)
                def tick():
                    motion['yaw'] += motion['speed']*0.05
                    message = Odometry()
                    message.header.stamp = wheel.get_clock().now().to_msg()
                    message.pose.pose.orientation.z = math.sin(motion['yaw']/2)
                    message.pose.pose.orientation.w = math.cos(motion['yaw']/2)
                    message.twist.twist.angular.z = motion['speed']
                    odom_publisher.publish(message)
                    executor.spin_once(timeout_sec=0.005)
                    self.actions.step()
                def wait(future):
                    deadline = time.monotonic()+10
                    while not future.done() and time.monotonic() < deadline:
                        tick()
                    self.assertTrue(future.done())
                    return future.result()
                # Wait for fresh odometry and subscription graph discovery.
                deadline = time.monotonic()+0.3
                while time.monotonic() < deadline:
                    tick()
                handle = wait(client.send_goal_async(Turn.Goal(angle=0.3)))
                self.assertTrue(handle.accepted)
                response = wait(handle.get_result_async())
                self.assertTrue(response.result.success, response.result.message)
                for _ in range(10):
                    tick()
                self.assertAlmostEqual(abs(motion['yaw']), 0.3, delta=0.04)
                self.assertEqual(motion['commands'][-1], 0.0)
                self.assertTrue(any(value < 0 for value in motion['commands']))
                wheel.destroy_subscription(subscription)
                # Remove publisher so the next interface can be rediscovered.
                self.node.destroy_publisher(self.actions.publisher)
                self.actions.publisher = None
                deadline = time.monotonic()+0.2
                while time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=0.01)
        finally:
            client.destroy()
            executor.remove_node(wheel)
            wheel.destroy_node()
            executor.remove_node(self.node)
            executor.shutdown()

    def test_real_ros_action_discovery_defaults_and_cancel(self):
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)
        client_node = rclpy.create_node('look_turn_test_client')
        executor.add_node(client_node)
        clients = {name: ActionClient(client_node, kind, '/' + name)
                   for name, kind in self.actions.TYPES.items()}

        def wait(future, timeout=5):
            deadline = time.monotonic()+timeout
            while not future.done() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.01)
                self.actions.step()
            self.assertTrue(future.done(), 'ROS action timed out')
            return future.result()

        try:
            for client in clients.values():
                self.assertTrue(client.wait_for_server(timeout_sec=5))
            for name in ('look_left', 'look_right', 'look_ahead'):
                kind = self.actions.TYPES[name]
                handle = wait(clients[name].send_goal_async(kind.Goal()))
                self.assertTrue(handle.accepted)
                response = wait(handle.get_result_async())
                self.assertTrue(response.result.success)
            # Turn has no odometry/controller here: accepted, meaningful failure.
            handle = wait(clients['turn_left'].send_goal_async(Turn.Goal()))
            self.assertTrue(handle.accepted)
            self.assertFalse(wait(handle.get_result_async()).result.success)
            # Suspend stepping while submitting cancellation, then resume.
            original_step = self.actions.step
            self.actions.step = lambda: None
            handle = wait(clients['look_left'].send_goal_async(Look.Goal()))
            self.assertTrue(handle.accepted)
            cancel = wait(handle.cancel_goal_async())
            self.assertTrue(cancel.goals_canceling)
            self.actions.step = original_step
            response = wait(handle.get_result_async())
            self.assertFalse(response.result.success)
            self.assertEqual(response.status, module.GoalStatus.STATUS_CANCELED)
        finally:
            for client in clients.values():
                client.destroy()
            executor.remove_node(client_node)
            client_node.destroy_node()
            executor.remove_node(self.node)
            executor.shutdown()


if __name__ == '__main__':
    unittest.main()
