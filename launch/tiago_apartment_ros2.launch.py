"""Run the apartment TIAGo++ with the installed Jazzy Webots ROS 2 driver."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import Ros2SupervisorLauncher
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    project = Path(__file__).resolve().parents[1]
    tiago_resource = Path(get_package_share_directory('webots_ros2_tiago')) / 'resource'

    webots = ExecuteProcess(
        cmd=[
            '/usr/local/webots/webots',
            '--batch',
            '--port=1234',
            '--mode=realtime',
            str(project / 'worlds' / 'complete_apartment_tiago_ros2.wbt'),
        ],
        output='screen',
    )
    driver = WebotsController(
        robot_name='TIAGo',
        parameters=[
            {
                'robot_description': str(tiago_resource / 'tiago_webots.urdf'),
                'use_sim_time': True,
                'set_robot_state_publisher': True,
            },
            str(project / 'config' / 'ros2_control_wheel_odom.yml'),
        ],
        remappings=[
            ('/diffdrive_controller/cmd_vel', '/cmd_vel'),
            ('/diffdrive_controller/odom', '/wheel/odom'),
        ],
        respawn=False,
    )

    spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--controller-manager-timeout', '500'],
            output='screen',
        )
        for name in ('diffdrive_controller', 'joint_state_broadcaster')
    ]
    return LaunchDescription([
        webots,
        Ros2SupervisorLauncher(),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': '<robot name=""><link name=""/></robot>'}],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
            output='screen',
        ),
        driver,
        WaitForControllerConnection(target_driver=driver, nodes_to_start=spawners),
    ])
