#!/usr/bin/env python3
"""Small programmatic client: x y yaw [frame_id], with map as default."""

import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from navigate_to_position.action import NavigateToPosition


def main():
    if len(sys.argv) not in (4, 5):
        raise SystemExit('Usage: navigate_to_position_client X Y YAW [FRAME_ID]')
    rclpy.init()
    node = Node('navigate_to_position_client')
    client = ActionClient(node, NavigateToPosition, '/navigate_to_position')
    try:
        if not client.wait_for_server(timeout_sec=5.0):
            raise SystemExit('/navigate_to_position is unavailable')
        goal = NavigateToPosition.Goal()
        goal.x, goal.y, goal.yaw = map(float, sys.argv[1:4])
        goal.frame_id = sys.argv[4] if len(sys.argv) == 5 else 'map'

        # Action goals are asynchronous: wait first for acceptance, then for the result.
        sent = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, sent)
        handle = sent.result()
        if not handle.accepted:
            raise SystemExit('Goal rejected')
        finished = handle.get_result_async()
        rclpy.spin_until_future_complete(node, finished)
        print(finished.result().result)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
