#!/usr/bin/env python3
"""Check manifest-declared ROS packages, entry points and selected configuration."""
import importlib
import json
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

from webots_path import resolve_webots


def check(profile, arguments):
    from ament_index_python.packages import get_package_prefix
    import yaml
    root = Path(__file__).resolve().parents[1]
    paths = [root / 'simulation_actions', root / 'dependencies/basic']
    if profile in ('navigation', 'mapping'):
        paths += [root / 'navigate_to_position', root / 'dependencies/navigation']
    if profile == 'mapping':
        paths += [root / 'third_party/explore', root / 'third_party/explore_lite_msgs', root / 'dependencies/mapping']
    if profile in ('generic', 'multi-robot'):
        paths = [root / 'dependencies/generic']
    manifests = [ET.parse(path / 'package.xml').getroot() for path in paths]
    local = {manifest.findtext('name') for manifest in manifests}
    dependencies = {entry.text for manifest in manifests for entry in manifest
                    if entry.tag.endswith('depend')}
    errors = []
    for name in sorted(dependencies - local):
        if name.startswith('python3'):
            continue  # Non-ROS keys are checked by rosdep, imports below as well.
        try:
            get_package_prefix(name)
        except LookupError:
            errors.append(f'ROS package not discoverable: {name}')
    modules = ['rclpy', 'yaml', 'webots_ros2_driver.webots_controller',
               'webots_ros2_driver.webots_launcher',
               'webots_ros2_driver.wait_for_controller_connection']
    if profile in ('navigation', 'mapping'):
        modules += ['nav2_msgs.action', 'nav2_common.launch', 'numpy', 'PIL.Image']
    if profile in ('generic', 'multi-robot'):
        modules = ['rclpy', 'launch', 'sensor_msgs.msg', 'std_msgs.msg', 'rosgraph_msgs.msg',
                   'webots_ros2_driver.utils']
    for module in modules:
        try:
            importlib.import_module(module)
        except ImportError as error:
            errors.append(f'Cannot import {module}: {error}')
    executables = [('controller_manager', 'spawner'),
                   ('robot_state_publisher', 'robot_state_publisher'),
                   ('tf2_ros', 'static_transform_publisher')]
    if profile in ('navigation', 'mapping'):
        executables += [('laser_filters', 'scan_to_scan_filter_chain'),
                        ('nav2_map_server', 'map_server'), ('rviz2', 'rviz2')]
    if profile in ('generic', 'multi-robot'):
        executables = []
    for package, name in executables:
        try:
            executable = Path(get_package_prefix(package)) / 'lib' / package / name
            if not os.access(executable, os.X_OK):
                errors.append(f'Executable unavailable: {executable}')
        except LookupError:
            pass  # Already reported above.
    try:
        executable, version = resolve_webots()
        print(f'Webots {version}: {executable}')
    except (ImportError, RuntimeError, ValueError, IndexError) as error:
        errors.append(str(error))
    overrides = dict(argument.split(':=', 1) for argument in arguments)
    world = 'complete_apartment_tiago_mapping.wbt' if profile == 'mapping' else 'setting_the_table_complete_apartment_tiago_ros2.wbt'
    urdf = 'tiago_webots_mapping.urdf' if profile == 'mapping' else 'tiago_webots_wheels.urdf'
    files = [Path(overrides.get('world', root / 'worlds' / world)),
             Path(overrides.get('robot_urdf', root / 'config' / urdf)),
             Path(overrides.get('look_turn_config', root / 'config/look_turn.yaml')),
             root / 'config/ros2_control_wheel_odom.yml', root / 'config/tiago_arm_tuck.py',
             root / 'controllers/fallback_action_supervisor/fallback_action_supervisor.py']
    if profile in ('navigation', 'mapping'):
        files += [Path(overrides.get('params_file', root / 'nav2_params_jazzy.yaml'))]
    if profile == 'navigation':
        files += [Path(overrides.get('map', root / 'maps/kitchen.yaml'))]
    if profile == 'mapping':
        files += [Path(overrides.get('doors_config', root / 'config/apartment_mapping_doors.json')),
                  root / 'config/apartment_slam.yaml', root / 'config/apartment_explore.yaml']
    if profile in ('generic', 'multi-robot'):
        default_world = 'generic_multi_robot_apartment.wbt' if profile == 'multi-robot' else 'generic_empty.wbt'
        files = [Path(overrides.get('world', root / 'worlds' / default_world)),
                 root / 'controllers/world_clock/world_clock.py',
                 root / 'controllers/generic_robot/generic_robot.py',
                 root / 'config/generic_robot.urdf']
    for path in files:
        try:
            content = path.read_text()
            if path.suffix == '.urdf':
                ET.fromstring(content)
            elif path.suffix == '.json':
                json.loads(content)
            elif path.suffix in ('.yaml', '.yml'):
                data = yaml.safe_load(content)
                if path.name == 'kitchen.yaml' or path == Path(overrides.get('map', '')):
                    image = path.parent / data['image']
                    if not image.is_file():
                        errors.append(f'Map image missing: {image}')
        except (OSError, ValueError, KeyError, TypeError, ET.ParseError, yaml.YAMLError) as error:
            errors.append(f'Configuration unavailable/invalid: {path}: {error}')
    if errors:
        print('Unavailable prerequisites:\n  ' + '\n  '.join(errors), file=sys.stderr)
        return 2
    print(f'PASS: {profile} package discovery, executable/configuration checks, Webots detection')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(check(sys.argv[1], sys.argv[2:]))
    except ImportError as error:
        print(f'ROS environment prerequisite unavailable: {error}', file=sys.stderr)
        sys.exit(2)
