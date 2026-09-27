"""Accelerated ROS-clock test: long progress, then a real stall and next goal.

Run in an unused ROS_DOMAIN_ID; uses the actual compiled explorer and fake Nav2.
"""
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import unittest

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster
from test_mapping_explorer import test_map


class NavigationProgressIntegration(unittest.TestCase):
    def test_long_goal_progress_stall_cancel_and_continue(self):
        rclpy.init()
        node = rclpy.create_node('progress_test_servers')
        group = ReentrantCallbackGroup()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        map_pub = node.create_publisher(OccupancyGrid, '/map', qos)
        clock_pub = node.create_publisher(Clock, '/clock', 10)
        tf = TransformBroadcaster(node)
        map_pub.publish(test_map())
        state = dict(time=10, start=None, target=None, x=0.0, y=0.0,
                     goals=0, cancelled_at=None, succeeded=False)
        done = threading.Event()

        def tick():
            state['time'] += 1
            t = state['time']
            clock = Clock()
            clock.clock.sec = t
            clock_pub.publish(clock)
            if state['start'] is not None:
                age = t - state['start']
                g = state['target']
                length = math.hypot(g.x, g.y)
                travel = min(age, 140) * 0.01
                state['x'], state['y'] = g.x * travel / length, g.y * travel / length
            transform = TransformStamped()
            transform.header.stamp.sec = t
            transform.header.frame_id = 'map'
            transform.child_frame_id = 'base_link'
            transform.transform.translation.x = state['x']
            transform.transform.translation.y = state['y']
            transform.transform.rotation.w = 1.0
            tf.sendTransform(transform)

        def plan(handle):
            result = ComputePathToPose.Result()
            result.path.header.frame_id = 'map'
            result.path.poses = [handle.request.goal]
            handle.succeed()
            return result

        def navigate(handle):
            state['goals'] += 1
            if state['goals'] == 1:
                state['start'] = state['time']
                state['target'] = handle.request.pose.pose.position
                while not done.is_set() and rclpy.ok():
                    if handle.is_cancel_requested:
                        state['cancelled_at'] = state['time'] - state['start']
                        handle.canceled()
                        return NavigateToPose.Result()
                    feedback = NavigateToPose.Feedback()
                    feedback.distance_remaining = math.hypot(
                        state['target'].x - state['x'], state['target'].y - state['y'])
                    handle.publish_feedback(feedback)
                    time.sleep(0.05)
                handle.abort()
            else:
                state['succeeded'] = True
                handle.succeed()
            return NavigateToPose.Result()

        timer = node.create_timer(0.05, tick, callback_group=group)
        planner = ActionServer(node, ComputePathToPose, '/compute_path_to_pose', plan,
                               callback_group=group)
        navigator = ActionServer(node, NavigateToPose, '/navigate_to_pose', navigate,
                                 callback_group=group,
                                 cancel_callback=lambda _: CancelResponse.ACCEPT)
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        thread = threading.Thread(target=executor.spin)
        thread.start()
        log_path = Path('/tmp/tiago-progress-regression.log')
        with log_path.open('w') as log:
            process = subprocess.Popen([
                'ros2', 'run', 'explore_lite', 'explore', '--ros-args',
                '-p', 'use_sim_time:=true', '-p', 'costmap_topic:=/map',
                '-p', 'planner_frequency:=10.0', '-p', 'progress_timeout:=30.0',
                '-p', 'progress_distance:=0.15', '-p', 'blacklist_radius:=1.0'],
                stdout=log, stderr=log, start_new_session=True)
            try:
                deadline = time.monotonic() + 30
                while time.monotonic() < deadline and not state['succeeded']:
                    time.sleep(0.05)
                self.assertTrue(state['succeeded'], log_path.read_text())
                # The last 15 cm milestone can precede the stop by up to 15 s.
                self.assertGreaterEqual(state['cancelled_at'], 155, log_path.read_text())
                self.assertLessEqual(state['cancelled_at'], 175, log_path.read_text())
                self.assertGreaterEqual(state['goals'], 2)
                self.assertIn('Navigation progress', log_path.read_text())
                self.assertIn('Navigation stalled', log_path.read_text())
                self.assertIn('Frontier blacklisted', log_path.read_text())
            finally:
                done.set()
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=10)
                executor.shutdown()
                thread.join(timeout=5)
                planner.destroy()
                navigator.destroy()
                node.destroy_timer(timer)
                node.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
