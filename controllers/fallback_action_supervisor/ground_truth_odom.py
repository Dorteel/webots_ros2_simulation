"""Publish TIAGo's Webots world pose as planar ROS odometry."""

from math import atan2, cos, sin

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from world_utils import get_node


class GroundTruthOdom:
    def __init__(self, supervisor, ros_node, robot_name="TIAGo", initial_map_pose=None):
        self.supervisor = supervisor
        self.robot = get_node(supervisor, robot_name, "robot")
        self.publisher = ros_node.create_publisher(Odometry, "/odom", 10)
        self.broadcaster = TransformBroadcaster(ros_node)
        self.next_publish = 0.0
        if initial_map_pose is not None:
            self.publish_map_origin(ros_node, initial_map_pose)

    def publish_map_origin(self, ros_node, initial_map_pose):
        """Anchor Webots world odometry to the known initial map pose once."""
        map_x, map_y, map_z, map_yaw = initial_map_pose
        odom_x, odom_y, odom_z = self.robot.getPosition()
        orientation = self.robot.getOrientation()
        odom_yaw = atan2(orientation[3], orientation[0])
        offset_yaw = map_yaw - odom_yaw
        c, s = cos(offset_yaw), sin(offset_yaw)

        transform = TransformStamped()
        transform.header.stamp = Time(seconds=self.supervisor.getTime()).to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "odom"
        transform.transform.translation.x = map_x - (c * odom_x - s * odom_y)
        transform.transform.translation.y = map_y - (s * odom_x + c * odom_y)
        transform.transform.translation.z = map_z - odom_z
        transform.transform.rotation.z = sin(offset_yaw / 2)
        transform.transform.rotation.w = cos(offset_yaw / 2)
        self.map_broadcaster = StaticTransformBroadcaster(ros_node)
        self.map_broadcaster.sendTransform(transform)

    def publish_if_due(self):
        now = self.supervisor.getTime()
        if now + 1e-9 < self.next_publish:
            return
        while self.next_publish <= now + 1e-9:
            self.next_publish += 0.05  # 20 Hz in simulation time.

        position = self.robot.getPosition()
        orientation = self.robot.getOrientation()
        velocity = self.robot.getVelocity()  # World-frame linear and angular velocity.
        yaw = atan2(orientation[3], orientation[0])
        heading_cos, heading_sin = cos(yaw), sin(yaw)

        odom = Odometry()
        odom.header.stamp = Time(seconds=now).to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y = position[:2]
        odom.pose.pose.position.z = position[2]
        odom.pose.pose.orientation.z = sin(yaw / 2)
        odom.pose.pose.orientation.w = cos(yaw / 2)
        odom.twist.twist.linear.x = heading_cos * velocity[0] + heading_sin * velocity[1]
        odom.twist.twist.linear.y = -heading_sin * velocity[0] + heading_cos * velocity[1]
        odom.twist.twist.angular.z = velocity[5]
        self.publisher.publish(odom)

        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = odom.child_frame_id
        transform.transform.translation.x = position[0]
        transform.transform.translation.y = position[1]
        transform.transform.translation.z = position[2]
        transform.transform.rotation = odom.pose.pose.orientation
        self.broadcaster.sendTransform(transform)
