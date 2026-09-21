"""Start the TIAGo apartment with optional navigation."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = Path(__file__).resolve().parent
    navigation = LaunchConfiguration('navigation')
    return LaunchDescription([
        DeclareLaunchArgument('navigation', default_value='true'),
        # navigation.launch.py already includes the apartment, so include exactly one stack.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / 'navigation.launch.py')),
            condition=IfCondition(navigation),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / 'tiago_apartment_ros2.launch.py')),
            condition=UnlessCondition(navigation),
        ),
    ])
