#!/usr/bin/env python3
"""Wait for active Nav2 lifecycle nodes before starting automatic exploration."""
import time

import rclpy
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState


def main():
    rclpy.init(args=[])
    node = rclpy.create_node('wait_for_mapping_navigation')
    clients = {name: node.create_client(GetState, f'/{name}/get_state') for name in
               ('controller_server', 'planner_server', 'behavior_server', 'bt_navigator')}
    node.get_logger().info('Waiting for Nav2 lifecycle activation before exploration')
    deadline = time.monotonic() + 180.0
    result = 1
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            active = True
            for client in clients.values():
                if not client.wait_for_service(timeout_sec=0.2):
                    active = False
                    continue
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
                if not future.done():
                    client.remove_pending_request(future)
                    active = False
                elif future.exception() is not None:
                    active = False
                elif future.result().current_state.id != State.PRIMARY_STATE_ACTIVE:
                    active = False
            if active:
                node.get_logger().info('Nav2 active; starting frontier exploration')
                result = 0
                break
            rclpy.spin_once(node, timeout_sec=0.2)
        if result:
            node.get_logger().error('Nav2 did not activate; exploration not started')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
