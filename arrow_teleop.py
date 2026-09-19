#!/usr/bin/env python3
"""Drive TIAGo with arrow keys; release the keys to stop."""

import curses
import time

import rclpy
from geometry_msgs.msg import TwistStamped


PERIOD = 0.05  # 20 Hz
HOLD_TIME = 0.2


def publish_velocity(node, publisher, linear, angular):
    command = TwistStamped()
    command.header.stamp = node.get_clock().now().to_msg()
    command.header.frame_id = "base_link"
    command.twist.linear.x = linear
    command.twist.angular.z = angular
    publisher.publish(command)


def teleop(window, node, publisher):
    curses.curs_set(0)
    window.keypad(True)
    window.nodelay(True)
    window.addstr(0, 0, "Arrows: drive/turn  |  q or Esc: quit")
    window.refresh()

    linear = angular = 0.0
    linear_until = angular_until = 0.0
    while True:
        started = time.monotonic()
        rclpy.spin_once(node, timeout_sec=0)

        # Keep each axis active for 0.2 s after its most recent key press.
        while (key := window.getch()) != -1:
            if key in (ord("q"), 27):
                return
            if key == curses.KEY_UP:
                linear, linear_until = 0.5, started + HOLD_TIME
            elif key == curses.KEY_DOWN:
                linear, linear_until = -0.5, started + HOLD_TIME
            elif key == curses.KEY_LEFT:
                angular, angular_until = 1.0, started + HOLD_TIME
            elif key == curses.KEY_RIGHT:
                angular, angular_until = -1.0, started + HOLD_TIME

        now = time.monotonic()
        publish_velocity(
            node, publisher,
            linear if now < linear_until else 0.0,
            angular if now < angular_until else 0.0,
        )
        time.sleep(max(0.0, PERIOD - (time.monotonic() - started)))


def main():
    rclpy.init()
    node = rclpy.create_node("arrow_teleop")
    publisher = node.create_publisher(TwistStamped, "/cmd_vel", 10)
    try:
        curses.wrapper(teleop, node, publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot even if the terminal or input loop fails.
        publish_velocity(node, publisher, 0.0, 0.0)
        rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
