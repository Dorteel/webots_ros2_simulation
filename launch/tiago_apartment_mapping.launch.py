"""Autonomous apartment mapping using the existing TIAGo driver and Nav2 config."""
import json
from pathlib import Path
from uuid import uuid4

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, ExecuteProcess, RegisterEventHandler, EmitEvent)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


class MappingRewrittenYaml(RewrittenYaml):
    """Also allow a list override; Nav2's stock converter only converts scalars."""
    def convert(self, value):
        return json.loads(value) if value.startswith('[') else super().convert(value)


def generate_launch_description():
    project = Path(__file__).resolve().parents[1]

    def include(path, **arguments):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                        launch_arguments=arguments.items())

    # Keep the existing stack and tuning. During mapping, clear the robot's
    # actual 0.28 m body footprint to connect the lidar's near-field blind area.
    nav_params = MappingRewrittenYaml(
        source_file=str(project / 'nav2_params_jazzy.yaml'),
        param_rewrites={
            'robot_radius': '0.28',
            # Mapping already filters /scan_raw -> /scan; preserve that pipeline.
            'amcl.ros__parameters.scan_topic': '/scan',
            'local_costmap.local_costmap.ros__parameters.voxel_layer.scan.topic': '/scan',
            'global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.topic': '/scan',
            'collision_monitor.ros__parameters.scan.topic': '/scan',
            # Static first; otherwise it erases inflated costs used by DWB.
            'local_costmap.local_costmap.ros__parameters.plugins':
                '["static_layer", "voxel_layer", "inflation_layer"]',
            'local_costmap.local_costmap.ros__parameters.inflation_layer.inflation_radius': '0.30',
            'global_costmap.global_costmap.ros__parameters.static_layer.footprint_clearing_enabled': 'true',
            'local_costmap.local_costmap.ros__parameters.static_layer.footprint_clearing_enabled': 'true',
        }, convert_types=True)

    status_topic = '/mapping/doors_ready_' + uuid4().hex
    door_waiter = ExecuteProcess(
        cmd=['python3', str(project / 'scripts/wait_for_mapping_doors.py'),
             '--topic', status_topic], output='screen')
    navigation_waiter = ExecuteProcess(
        cmd=['python3', str(project / 'scripts/wait_for_mapping_navigation.py')],
        condition=IfCondition(LaunchConfiguration('explore')), output='screen')
    explorer = Node(package='explore_lite', executable='explore', name='explore_node',
             parameters=[str(project / 'config/apartment_explore.yaml')],
             condition=IfCondition(LaunchConfiguration('explore')), output='screen')

    def navigation_finished(event, context):
        if event.returncode == 0:
            return [explorer]
        return [EmitEvent(event=Shutdown(
            reason='Nav2 activation failed; frontier exploration not started'))]

    mapping_nodes = [
        Node(package='nav2_map_server', executable='map_saver_server',
             name='mapping_map_saver', output='screen',
             condition=IfCondition(LaunchConfiguration('explore')),
             parameters=[{'use_sim_time': True, 'save_map_timeout': 15.0,
                          'map_subscribe_transient_local': True}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_mapping_save', output='screen',
             condition=IfCondition(LaunchConfiguration('explore')),
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': ['mapping_map_saver']}]),
        ExecuteProcess(cmd=['python3', str(project / 'scripts/mapping_finalizer.py'),
                            '--ros-args', '-p', 'use_sim_time:=true',
                            '-p', ['maps_directory:=', LaunchConfiguration('maps_directory')],
                            '-p', ['world:=', LaunchConfiguration('world')],
                            '-p', ['scene_graph:="', LaunchConfiguration('scene_graph'), '"']],
                       condition=IfCondition(LaunchConfiguration('explore')), output='screen'),
        include(Path(get_package_share_directory('slam_toolbox')) /
                'launch/online_async_launch.py', use_sim_time='true',
                slam_params_file=str(project / 'config/apartment_slam.yaml')),
        include(project / 'launch/mapping_navigation.launch.py',
                use_sim_time='true', autostart='true',
                params_file=nav_params),
        RegisterEventHandler(OnProcessExit(target_action=navigation_waiter,
                                            on_exit=navigation_finished)),
        navigation_waiter,
    ]

    def doors_finished(event, context):
        if event.returncode == 0:
            return mapping_nodes
        return [EmitEvent(event=Shutdown(
            reason='Apartment door preparation failed; SLAM/exploration not started'))]

    return LaunchDescription([
        DeclareLaunchArgument('doors_config', default_value=str(
            project / 'config/apartment_mapping_doors.json')),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('maps_directory', default_value=str(project / 'maps')),
        DeclareLaunchArgument('scene_graph', default_value=''),
        DeclareLaunchArgument('explore', default_value='true'),
        DeclareLaunchArgument('world', default_value=str(
            project / 'worlds/complete_apartment_tiago_mapping.wbt')),
        DeclareLaunchArgument('robot_urdf', default_value=str(
            project / 'config/tiago_webots_mapping.urdf')),
        # SLAM alone owns map -> odom, even if the caller exported a static anchor.
        SetEnvironmentVariable('TIAGO_INITIAL_MAP_POSE', ''),
        include(project / 'launch/tiago_apartment_ros2.launch.py',
                world=LaunchConfiguration('world'),
                robot_urdf=LaunchConfiguration('robot_urdf'), scan_topic='/scan_raw',
                mapping_doors_config=LaunchConfiguration('doors_config'),
                mapping_doors_status_topic=status_topic),
        RegisterEventHandler(OnProcessExit(target_action=door_waiter, on_exit=doors_finished)),
        door_waiter,
        Node(package='laser_filters', executable='scan_to_scan_filter_chain',
             name='scan_to_scan_filter_chain',
             parameters=[str(project / 'config/apartment_scan_filter.yaml')],
             remappings=[('scan', '/scan_raw'), ('scan_filtered', '/scan')],
             output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(project / 'config/apartment_mapping.rviz')],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
