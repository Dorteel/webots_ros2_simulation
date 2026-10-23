#!/usr/bin/env python3
"""Bounded smoke check for the default full robot; requires an isolated DDS domain."""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from rosgraph_msgs.msg import Clock


def main():
    rclpy.init()
    node = Node('simulation_smoke_check', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    received = {}
    counts = {}
    def record(key, message):
        received[key] = message
        counts[key] = counts.get(key, 0) + 1
    subscriptions = [node.create_subscription(kind, topic,
        lambda msg, key=key: record(key, msg), qos_profile_sensor_data)
        for key, kind, topic in [
            ('rgb', Image, '/tiago/camera/color/image_raw'),
            ('depth', Image, '/tiago/camera/depth/image_raw'),
            ('odom', Odometry, '/odom'), ('clock', Clock, '/clock')]]
    publisher = None
    stamped = False
    def spin_until(condition, seconds, description):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if condition():
                return
        raise RuntimeError('Timeout: ' + description)
    def send(speed):
        message = TwistStamped() if stamped else Twist()
        twist = message.twist if stamped else message
        twist.angular.z = speed
        if stamped:
            message.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(message)
    def yaw():
        q = received['odom'].pose.pose.orientation
        return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    try:
        required = {'controller_manager', 'robot_state_publisher', 'fallback_ground_truth_odom'}
        spin_until(lambda: required <= set(node.get_node_names()) and
                   all(counts.get(key, 0) >= 3 for key in ('rgb', 'depth', 'odom', 'clock')),
                   100, 'required nodes and repeated RGB/depth/odom/clock messages')
        for key, encoding in [('rgb', 'bgra8'), ('depth', '32FC1')]:
            image = received[key]
            assert image.width > 0 and image.height > 0 and image.data, key + ' empty'
            assert image.encoding == encoding, key + ' unexpected encoding'
            assert image.step >= image.width * 4, key + ' invalid row stride'
            assert len(image.data) == image.step * image.height, key + ' truncated image'
        before = node.get_clock().now().nanoseconds
        spin_until(lambda: node.get_clock().now().nanoseconds > before + 100_000_000,
                   10, 'advancing simulation time')
        for key in ('rgb', 'depth', 'odom'):
            stamp = received[key].header.stamp
            seconds = stamp.sec + stamp.nanosec * 1e-9
            assert abs(node.get_clock().now().nanoseconds * 1e-9 - seconds) < 1, key + ' timestamp mismatch'
        # Sensors can arrive before the controller spawners finish activating.
        # Wait for the velocity endpoint instead of failing this startup race.
        spin_until(lambda: bool(dict(node.get_topic_names_and_types()).get('/cmd_vel')),
                   30, 'wheel controller command topic')
        topics = dict(node.get_topic_names_and_types())
        types = topics.get('/cmd_vel', [])
        assert len(types) == 1 and types[0] in (
            'geometry_msgs/msg/Twist', 'geometry_msgs/msg/TwistStamped'), str(types)
        stamped = types[0].endswith('TwistStamped')
        publisher = node.create_publisher(TwistStamped if stamped else Twist, '/cmd_vel', 10)
        spin_until(lambda: publisher.get_subscription_count() > 0, 10, 'velocity subscriber')
        initial_yaw = yaw()
        position = received['odom'].pose.pose.position
        initial_xy = (position.x, position.y)
        # Rotate slowly in place for at most 2 wall seconds; always stop in finally.
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            send(0.1)
            rclpy.spin_once(node, timeout_sec=0.05)
            delta = math.atan2(math.sin(yaw()-initial_yaw), math.cos(yaw()-initial_yaw))
            if delta > 0.02:
                break
        send(0.0)
        stopped_after = counts['odom']
        assert delta > 0.005, 'No measured rotation after command'
        position = received['odom'].pose.pose.position
        assert math.hypot(position.x-initial_xy[0], position.y-initial_xy[1]) < 0.08, 'Unexpected translation'
        spin_until(lambda: counts['odom'] > stopped_after + 2 and
                   abs(received['odom'].twist.twist.angular.z) < 0.02,
                   5, 'robot stopped')
        print('PASS: required nodes, RGB/depth data, advancing synchronized time, rotation and stop')
    finally:
        if publisher is not None:
            for _ in range(5):
                send(0.0)
                rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
