"""Start apartment simulation, asynchronous SLAM, and a mapping RViz view."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    project = Path(__file__).resolve().parents[1]
    tiago_share = Path(get_package_share_directory('webots_ros2_tiago'))
    slam_share = Path(get_package_share_directory('slam_toolbox'))

    # Mapping uses the same apartment and controls, with the base-only robot.
    apartment = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(project / 'launch' / 'tiago_apartment_ros2.launch.py')),
        launch_arguments={
            'world': str(project / 'worlds' / 'complete_apartment_tiago_mapping.wbt'),
            'robot_urdf': str(project / 'config' / 'tiago_webots_mapping.urdf'),
        }.items(),
    )
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(slam_share / 'launch' / 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': str(tiago_share / 'resource' / 'slam_toolbox_params.yaml'),
        }.items(),
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', str(project / 'config' / 'mapping.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    return LaunchDescription([apartment, mapping, rviz])
