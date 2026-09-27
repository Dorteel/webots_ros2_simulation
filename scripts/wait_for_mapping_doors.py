#!/usr/bin/env python3
"""Exit successfully only after this launch's Supervisor reports settled doors."""
import argparse
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', required=True)
    parser.add_argument('--timeout', type=float, default=180.0)
    args = parser.parse_args()
    rclpy.init(args=[])
    node = rclpy.create_node('wait_for_mapping_doors')
    state = []
    subscription = node.create_subscription(
        String, args.topic, lambda message: state.append(message.data),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.get_logger().info('Waiting for apartment doors to open and settle before SLAM')
    deadline = time.monotonic() + args.timeout
    result = 1
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if state and state[-1].startswith('ready: '):
                node.get_logger().info('Apartment doors open and settled (radians): ' + state[-1][7:])
                node.get_logger().info('Apartment doors ready; starting SLAM and navigation')
                result = 0
                break
            if state and state[-1].startswith('error:'):
                node.get_logger().error(state[-1])
                break
        else:
            node.get_logger().error('Timed out waiting for mapping door preparation')
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
