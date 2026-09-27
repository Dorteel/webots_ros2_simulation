"""Start apartment navigation with Webots ground-truth localization."""

from pathlib import Path
from uuid import uuid4

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, SetEnvironmentVariable,
                            DeclareLaunchArgument, OpaqueFunction, ExecuteProcess,
                            RegisterEventHandler, EmitEvent)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _launch(context):
    project = Path(__file__).resolve().parents[1]
    params_file = Path(LaunchConfiguration('params_file').perform(context))
    map_file = Path(LaunchConfiguration('map').perform(context))
    nav2_launch = Path(get_package_share_directory('nav2_bringup')) / 'launch' / 'navigation_launch.py'
    initial = yaml.safe_load(params_file.read_text())['amcl']['ros__parameters']['initial_pose']
    initial_pose = ','.join(str(initial.get(axis, 0.0)) for axis in ('x', 'y', 'z', 'yaw'))

    # The Supervisor anchors map -> odom to this pose and owns odom -> base_link.
    map_origin = SetEnvironmentVariable('TIAGO_INITIAL_MAP_POSE', initial_pose)
    extra = []
    transform = LaunchConfiguration('map_to_odom').perform(context)
    if transform:
        x, y, yaw = map(float, transform.split(','))
        map_origin = SetEnvironmentVariable('TIAGO_INITIAL_MAP_POSE', '')
        extra.append(Node(package='tf2_ros', executable='static_transform_publisher',
                          arguments=['--x', str(x), '--y', str(y), '--yaw', str(yaw),
                                     '--frame-id', 'map', '--child-frame-id', 'odom']))
    doors = LaunchConfiguration('doors_config').perform(context)
    status_topic = '/cmoc/doors_' + uuid4().hex
    apartment = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(project / 'launch' / 'tiago_apartment_ros2.launch.py')
    ), launch_arguments={'mapping_doors_config': doors,
                         'mapping_doors_status_topic': status_topic}.items())
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
    position_action = Node(
        package='navigate_to_position', executable='navigate_to_position_server',
        parameters=[{'use_sim_time': True}], output='screen',
    )
    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', str(project / 'config' / 'navigation.rviz')],
        parameters=[{'use_sim_time': True}],
    )
    navigation_nodes = [map_server, map_lifecycle, navigation, position_action, rviz]
    nodes = navigation_nodes
    if doors:
        waiter = ExecuteProcess(cmd=['python3', str(project / 'scripts/wait_for_mapping_doors.py'),
                                     '--topic', status_topic], output='screen')

        def doors_finished(event, context):
            if event.returncode == 0:
                return navigation_nodes
            return [EmitEvent(event=Shutdown(reason='Door preparation failed'))]

        nodes = [RegisterEventHandler(OnProcessExit(target_action=waiter,
                                                    on_exit=doors_finished)), waiter]
    return [map_origin, *extra, apartment, *nodes]


def generate_launch_description():
    project = Path(__file__).resolve().parents[1]
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=str(project / 'maps/kitchen.yaml')),
        DeclareLaunchArgument('params_file', default_value=str(project / 'nav2_params_jazzy.yaml')),
        DeclareLaunchArgument('map_to_odom', default_value='',
                              description='Optional verified x,y,yaw transform for world odometry'),
        DeclareLaunchArgument('doors_config', default_value=''),
        OpaqueFunction(function=_launch),
    ])
