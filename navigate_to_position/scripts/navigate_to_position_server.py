#!/usr/bin/env python3
"""Expose simple x/y/yaw goals through Nav2's NavigateToPose action."""

import math
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from navigate_to_position.action import NavigateToPosition


class NavigateToPositionServer(Node):
    def __init__(self):
        super().__init__('navigate_to_position_server')
        # Both actions need callback time while a goal is in progress.
        callbacks = ReentrantCallbackGroup()
        self.nav2 = ActionClient(self, NavigateToPose, '/navigate_to_pose',
                                 callback_group=callbacks)
        self.server = ActionServer(
            self, NavigateToPosition, '/navigate_to_position',
            execute_callback=self.execute,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=callbacks,
        )

    def execute(self, goal_handle):
        result = NavigateToPosition.Result()
        request = goal_handle.request
        pose = PoseStamped()
        pose.header.frame_id = request.frame_id or 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = request.x
        pose.pose.position.y = request.y
        pose.pose.orientation.z = math.sin(request.yaw / 2.0)
        pose.pose.orientation.w = math.cos(request.yaw / 2.0)

        try:
            # Wait briefly for Nav2; return a normal action failure if it is absent.
            for _ in range(25):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.message = 'Canceled before Nav2 accepted the goal'
                    return result
                if self.nav2.wait_for_server(timeout_sec=0.2):
                    break
            else:
                goal_handle.abort()
                result.message = 'Nav2 /navigate_to_pose is unavailable'
                return result

            def feedback(nav_feedback):
                if goal_handle.is_active and not goal_handle.is_cancel_requested:
                    update = NavigateToPosition.Feedback()
                    update.distance_remaining = nav_feedback.feedback.distance_remaining
                    goal_handle.publish_feedback(update)

            nav_future = self.nav2.send_goal_async(
                NavigateToPose.Goal(pose=pose), feedback_callback=feedback)
            while not nav_future.done():
                time.sleep(0.1)
            nav_goal = nav_future.result()
            if not nav_goal.accepted:
                goal_handle.abort()
                result.message = 'Nav2 rejected the goal'
                return result

            nav_result = nav_goal.get_result_async()
            while not nav_result.done():
                if goal_handle.is_cancel_requested:
                    # Forward cancellation to the actual Nav2 goal.
                    cancel = nav_goal.cancel_goal_async()
                    for _ in range(20):
                        if cancel.done():
                            break
                        time.sleep(0.1)
                    goal_handle.canceled()
                    result.message = 'Canceled'
                    return result
                time.sleep(0.1)

            response = nav_result.result()
            result.success = response.status == GoalStatus.STATUS_SUCCEEDED
            result.message = 'Goal reached' if result.success else f'Nav2 finished with status {response.status}'
            if result.success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
        except Exception as exc:
            goal_handle.abort()
            result.message = f'Nav2 request failed: {exc}'
        return result


def main():
    rclpy.init()
    node = NavigateToPositionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        rclpy.spin(node, executor=executor)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
