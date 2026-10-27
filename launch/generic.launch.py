"""Generic worlds with one independent native Webots driver per robot."""
import atexit
import importlib.util
import json
import math
import random
import re
import secrets
import tempfile
from pathlib import Path
import socket
import sys

from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            OpaqueFunction, RegisterEventHandler, EmitEvent, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from webots_ros2_driver.webots_controller import WebotsController


def randomized_world(world, seed_text):
    """Write a launch-local copy before physics starts; never move a live robot.

    Bounds are approved center regions declared by the world author, not an
    automatic collision detector. Keep the copy beside its source so relative
    PROTO, texture and controller paths continue to resolve.
    """
    source = world.read_text()
    marker = re.search(r'^# GENERIC_SPAWN_REGION (.+)$', source, re.MULTILINE)
    if marker is None:
        raise ValueError('Multi-robot world must declare a validated GENERIC_SPAWN_REGION')
    region = json.loads(marker.group(1))
    for axis in ('x', 'y'):
        bounds = region[axis]
        if len(bounds) != 2 or not all(math.isfinite(v) for v in bounds) or bounds[0] > bounds[1]:
            raise ValueError('Spawn center bounds must be finite ordered pairs')
    if not math.isfinite(region['z']):
        raise ValueError('Spawn height must be finite')
    seed = int(seed_text) if seed_text else secrets.randbits(32)
    rng = random.Random(seed)
    x, y = rng.uniform(*region['x']), rng.uniform(*region['y'])
    yaw = rng.uniform(-math.pi, math.pi)
    pattern = r'(DEF TURTLEBOT TurtleBot3Burger \{\n)  translation [^\n]+\n  rotation [^\n]+'
    source, count = re.subn(pattern, lambda m: m[1] +
                           f'  translation {x:.12f} {y:.12f} {region["z"]}\n'
                           f'  rotation 0 0 1 {yaw:.12f}', source)
    if count != 1:
        raise ValueError('Expected one DEF TURTLEBOT with translation and rotation fields')
    with tempfile.NamedTemporaryFile(mode='w', prefix='.generic_spawn_', suffix='.wbt',
                                     dir=world.parent, delete=False) as stream:
        stream.write(source)
        generated = Path(stream.name)
    def cleanup():
        generated.unlink(missing_ok=True)
        generated.with_name('.' + generated.stem + '.wbproj').unlink(missing_ok=True)
    atexit.register(cleanup)
    print(f'TurtleBot spawn: seed={seed}, position=({x:.12f}, {y:.12f}, {region["z"]}), '
          f'yaw={yaw:.12f}; source={world}', flush=True)
    return generated


def start(context):
    root = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location('generic_webots_path', root / 'scripts/webots_path.py')
    paths = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(paths)
    executable, _ = paths.resolve_webots()
    sys.path.insert(0, str(root / 'controllers/generic_robot'))
    from names import robot_namespaces
    multi = LaunchConfiguration('scenario').perform(context) == 'multi-robot'
    world_arg = LaunchConfiguration('world').perform(context)
    world = Path(world_arg).resolve() if world_arg else root / 'worlds' / (
        'generic_multi_robot_apartment.wbt' if multi else 'generic_empty.wbt')
    if multi:
        world = randomized_world(world, LaunchConfiguration('seed').perform(context))
    robot_arg = LaunchConfiguration('robots').perform(context)
    names = json.loads(robot_arg) if robot_arg else (['TIAGo', 'TurtleBot'] if multi else [])
    namespaces = robot_namespaces(names)
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = str(probe.getsockname()[1])
    process = ExecuteProcess(cmd=[str(executable), '--batch', '--stdout', '--stderr',
                                  '--mode=realtime', '--port=' + port,
                                  str(world)], output='screen')
    drivers = [WebotsController(
        robot_name=name, namespace=namespace, port=port,
        remappings=[('__node', 'driver'), ('/remove_urdf_robot', 'remove_urdf_robot')],
        parameters=[{'robot_description': str(root / 'config/generic_robot.urdf'),
                     'use_sim_time': True, 'set_robot_state_publisher': False}],
    ) for name, namespace in namespaces.items()]
    return [SetEnvironmentVariable('GENERIC_ROBOT_NAMESPACES', json.dumps(namespaces)),
            AppendEnvironmentVariable('PYTHONPATH', str(root / 'controllers/generic_robot'), prepend=True),
            RegisterEventHandler(OnProcessExit(target_action=process,
                on_exit=[EmitEvent(event=Shutdown(reason='Webots exited'))])), process, *drivers]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='generic', choices=['generic', 'multi-robot']),
        DeclareLaunchArgument('world', default_value='', description='World override; defaults to scenario world'),
        DeclareLaunchArgument('seed', default_value='', description='Integer spawn seed; blank chooses and prints a random seed'),
        DeclareLaunchArgument('robots', default_value='', description='JSON list of Webots names (type shorthand) or {name, type} objects'),
        OpaqueFunction(function=start),
    ])
