"""
Contents
--------
main() - Starts the single Webots Supervisor loop.
"""

import os

from controller import Supervisor
import rclpy

from actions import execute_action
from command_server import close_server, open_server, process_commands
from ground_truth_odom import GroundTruthOdom


def main():
    """Keep the shared Supervisor alive for callers of execute_action()."""
    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep())
    server = open_server()
    rclpy.init(args=[])
    ros_node = rclpy.create_node("fallback_ground_truth_odom")

    try:
        pose_text = os.environ.get("TIAGO_INITIAL_MAP_POSE")
        initial_map_pose = tuple(map(float, pose_text.split(","))) if pose_text else None
        if initial_map_pose is not None and len(initial_map_pose) != 4:
            raise ValueError("TIAGO_INITIAL_MAP_POSE must contain x,y,z,yaw")
        odom = GroundTruthOdom(supervisor, ros_node, initial_map_pose=initial_map_pose)
        while supervisor.step(timestep) != -1:
            process_commands(server, supervisor, execute_action)
            odom.publish_if_due()
    finally:
        close_server(server)
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
