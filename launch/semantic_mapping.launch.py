"""Start the existing navigation stack for RViz semantic point annotation."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    navigation = Path(__file__).resolve().with_name('navigation.launch.py')
    # Keyboard prompts run in a separate terminal; ros2 launch does not own stdin.
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(navigation)))
    ])
