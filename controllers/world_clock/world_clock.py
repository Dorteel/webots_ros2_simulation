"""World clock only. No robot discovery, scene access, or actuator operations."""
from controller import Robot
import rclpy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock


def main():
    robot = Robot()
    rclpy.init(args=[])
    node = rclpy.create_node('world_clock')
    publisher = node.create_publisher(Clock, '/clock', 10)
    try:
        while rclpy.ok() and robot.step(int(robot.getBasicTimeStep())) != -1:
            publisher.publish(Clock(clock=Time(nanoseconds=round(robot.getTime() * 1e9)).to_msg()))
            rclpy.spin_once(node, timeout_sec=0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
