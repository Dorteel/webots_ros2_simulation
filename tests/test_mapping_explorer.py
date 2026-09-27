"""ROS integration regression test; run in an unused ROS_DOMAIN_ID.

Exercises the vendored executable against deterministic maps and action servers,
without Webots. This tests orchestration, not Nav2's real planner/controller.
"""
import os
import signal
import subprocess
import time
import unittest
from pathlib import Path

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from explore_lite_msgs.msg import ExploreStatus
from tf2_ros import StaticTransformBroadcaster


def test_map():
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.resolution = 0.1
    grid.info.width = grid.info.height = 60
    grid.info.origin.position.x = grid.info.origin.position.y = -3.0
    grid.info.origin.orientation.w = 1.0
    cells = [100] * 3600
    for y in range(10, 50):
        for x in range(10, 50):
            cells[y * 60 + x] = 0
    # Four disconnected unknown boundaries against the known central room.
    for y in range(60):
        for x in range(60):
            if ((25 <= x < 35 and (y < 10 or y >= 50)) or
                    (25 <= y < 35 and (x < 10 or x >= 50))):
                cells[y * 60 + x] = -1
    grid.data = cells
    return grid


class ExplorerIntegration(unittest.TestCase):
    def test_failed_frontiers_are_bounded_and_completion_is_published(self):
        rclpy.init()
        node = rclpy.create_node('explorer_test_servers')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publisher = node.create_publisher(OccupancyGrid, '/map', qos)
        tf = StaticTransformBroadcaster(node)
        transform = TransformStamped()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'base_link'
        transform.transform.rotation.w = 1.0
        tf.sendTransform(transform)
        grid = test_map()
        publisher.publish(grid)
        plans, navigation_requests, navigation_results, statuses = [], [], [], []
        completions = []
        completion_subscription = node.create_subscription(
            Bool, '/exploration_complete',
            lambda m: completions.append((m.data, len(plans), len(navigation_results))), qos)

        resume_pub = node.create_publisher(Bool, '/explore/resume', 10)

        def plan(handle):
            plans.append(handle.request.goal)
            result = ComputePathToPose.Result()
            if len(plans) == 1:
                handle.abort()
            else:
                result.path.header.frame_id = 'map'
                result.path.poses = [handle.request.goal]
                handle.succeed()
            return result

        def accept(goal):
            navigation_requests.append(goal)
            return GoalResponse.REJECT if len(navigation_requests) == 1 else GoalResponse.ACCEPT

        def navigate(handle):
            navigation_results.append(handle.request)
            if len(navigation_results) == 1:
                resume_pub.publish(Bool(data=False))
                handle.abort()
            else:
                handle.succeed()
            return NavigateToPose.Result()

        planner = ActionServer(node, ComputePathToPose, '/compute_path_to_pose', plan)
        navigator = ActionServer(node, NavigateToPose, '/navigate_to_pose', navigate,
                                 goal_callback=accept)
        subscription = node.create_subscription(
            ExploreStatus, '/explore/status', lambda m: statuses.append(m.status), qos)
        log_path = Path('/tmp/tiago-explorer-regression.log')
        with log_path.open('w') as log:
            process = subprocess.Popen([
                'ros2', 'run', 'explore_lite', 'explore', '--ros-args',
                '-p', 'costmap_topic:=/map', '-p', 'planner_frequency:=5.0',
                '-p', 'min_frontier_size:=0.5', '-p', 'blacklist_radius:=1.5'], stdout=log, stderr=log,
                start_new_session=True)
            try:
                resumed = False
                deadline = time.monotonic() + 30
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.1)
                    if ExploreStatus.EXPLORATION_PAUSED in statuses and not resumed:
                        self.assertFalse(any(c[0] for c in completions))
                        resumed = True
                        resume_pub.publish(Bool(data=True))
                    if ExploreStatus.EXPLORATION_COMPLETE in statuses:
                        break
                self.assertTrue(resumed)
                self.assertIn(ExploreStatus.EXPLORATION_COMPLETE, statuses,
                              log_path.read_text())
                # Completion is distinct from preceding rejected/aborted goals.
                until = time.monotonic() + 1
                while time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=0.05)
                self.assertEqual([c for c in completions if c[0]], [(True, 4, 2)])
                late = []
                late_subscription = node.create_subscription(
                    Bool, '/exploration_complete', lambda m: late.append(m.data), qos)
                until = time.monotonic() + 2
                while not late and time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=0.05)
                self.assertEqual(late, [True])
                node.destroy_subscription(late_subscription)
                self.assertEqual(len(plans), 4)
                self.assertEqual(len(navigation_requests), 3)
                self.assertEqual(len(navigation_results), 2)
                self.assertIn('Navigation goal succeeded', log_path.read_text())
                self.assertIn('Navigation goal failed', log_path.read_text())
            finally:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=10)
                planner.destroy()
                navigator.destroy()
                node.destroy_subscription(subscription)
                node.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
