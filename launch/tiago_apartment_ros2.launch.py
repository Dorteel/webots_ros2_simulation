"""Run the apartment TIAGo++ with the installed Jazzy Webots ROS 2 driver."""

from pathlib import Path
import socket

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, ExecuteProcess
from launch_ros.actions import Node
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import Ros2SupervisorLauncher
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    project = Path(__file__).resolve().parents[1]
    robot_urdf = project / 'config' / 'tiago_webots_wheels.urdf'
    # Give Webots and its external controllers the same free port, even if an older simulation is open.
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = str(probe.getsockname()[1])

    webots = ExecuteProcess(
        cmd=[
            '/usr/local/webots/webots',
            '--batch',
            f'--port={port}',
            '--mode=realtime',
            str(project / 'worlds' / 'complete_apartment_tiago_ros2.wbt'),
        ],
        output='screen',
    )
    driver = WebotsController(
        robot_name='TIAGo',
        port=port,
        parameters=[
            {
                'robot_description': str(robot_urdf),
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
        AppendEnvironmentVariable('PYTHONPATH', str(project / 'config'), prepend=True),
        webots,
        Ros2SupervisorLauncher(port=port),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_urdf.read_text()}],
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
