"""
Contents
--------
main() - Starts the single Webots Supervisor loop.
"""

import os
import sys

from controller import Supervisor
import rclpy

from actions import execute_action
from command_server import close_server, open_server, process_commands
from ground_truth_odom import GroundTruthOdom
from mapping_doors import MappingDoors
from pick_action_server import PickActionServer
from fallback_action_servers import FallbackActionServers


def main():
    """Keep the shared Supervisor alive for callers of execute_action()."""
    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep())
    try:
        server = open_server()
    except RuntimeError as error:
        print(str(error), file=sys.stderr, flush=True)
        return 1
    rclpy.init(args=[])
    ros_node = rclpy.create_node("fallback_ground_truth_odom")
    pick_server = PickActionServer(ros_node, supervisor)
    fallback_servers = FallbackActionServers(ros_node, supervisor)

    try:
        pose_text = os.environ.get("TIAGO_INITIAL_MAP_POSE")
        initial_map_pose = tuple(map(float, pose_text.split(","))) if pose_text else None
        if initial_map_pose is not None and len(initial_map_pose) != 4:
            raise ValueError("TIAGO_INITIAL_MAP_POSE must contain x,y,z,yaw")
        odom = GroundTruthOdom(supervisor, ros_node, initial_map_pose=initial_map_pose,
                               mapping_metadata=bool(os.environ.get('TIAGO_MAPPING_DOORS_CONFIG')))
        door_config = os.environ.get('TIAGO_MAPPING_DOORS_CONFIG', '')
        mapping_doors = MappingDoors(
            supervisor, ros_node, door_config,
            os.environ['TIAGO_MAPPING_DOORS_STATUS_TOPIC']) if door_config else None
        while supervisor.step(timestep) != -1:
            if mapping_doors is not None:
                mapping_doors.step()
            process_commands(server, supervisor, execute_action)
            rclpy.spin_once(ros_node, timeout_sec=0)
            odom.publish_if_due()
    finally:
        close_server(server)
        fallback_servers.destroy()
        pick_server.destroy()
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
