"""Start apartment navigation with Webots ground-truth localization."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    project = Path(__file__).resolve().parents[1]
    params_file = project / 'nav2_params_jazzy.yaml'
    map_file = project / 'maps' / 'kitchen.yaml'
    nav2_launch = Path(get_package_share_directory('nav2_bringup')) / 'launch' / 'navigation_launch.py'
    initial = yaml.safe_load(params_file.read_text())['amcl']['ros__parameters']['initial_pose']
    initial_pose = ','.join(str(initial.get(axis, 0.0)) for axis in ('x', 'y', 'z', 'yaw'))

    # The Supervisor anchors map -> odom to this pose and owns odom -> base_link.
    map_origin = SetEnvironmentVariable('TIAGO_INITIAL_MAP_POSE', initial_pose)
    apartment = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(project / 'launch' / 'tiago_apartment_ros2.launch.py')
    ))
    # Ground-truth TF replaces AMCL; the static map server has its own lifecycle.
    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        parameters=[str(params_file), {'yaml_filename': str(map_file), 'use_sim_time': True}],
        output='screen',
    )
    map_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map',
        parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['map_server']}],
        output='screen',
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav2_launch)),
        launch_arguments={
            'params_file': str(params_file),
            'use_sim_time': 'True',
            'autostart': 'True',
        }.items(),
    )
    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', str(project / 'config' / 'navigation.rviz')],
        parameters=[{'use_sim_time': True}],
    )
    return LaunchDescription([map_origin, apartment, map_server, map_lifecycle, navigation, rviz])
