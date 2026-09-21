#!/usr/bin/env python3
"""Compute one Nav2 path from explicit or semantic poses and save it as YAML."""

import argparse
from math import cos, sin
from pathlib import Path
import re

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
import yaml

from navigate_to_position.follow_saved_path_client import path_to_dict

ROOT = Path(__file__).resolve().parents[1]


def semantic_pose(filename, name):
    objects = yaml.safe_load(filename.read_text())
    entry = objects[name]
    pose = entry.get('preferred_navigation_pose') or entry.get('navigation_pose')
    if not pose:
        raise ValueError(f'{name} has no navigation pose')
    return pose


def stamped_pose(node, pose):
    message = PoseStamped()
    message.header.frame_id = pose.get('frame_id', 'map')
    message.header.stamp = node.get_clock().now().to_msg()
    message.pose.position.x = float(pose['x'])
    message.pose.position.y = float(pose['y'])
    yaw = float(pose['yaw'])
    message.pose.orientation.z = sin(yaw / 2)
    message.pose.orientation.w = cos(yaw / 2)
    return message


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('route_name', help='filename stem under config/paths/')
    parser.add_argument('--start', nargs=3, type=float, metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--goal', nargs=3, type=float, metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--from-object', dest='source')
    parser.add_argument('--to-object', dest='target')
    parser.add_argument('--semantic-map', type=Path,
                        default=ROOT / 'config/semantic_navigation_map.yaml')
    parser.add_argument('--frame-id', default='map', help='frame for numeric poses')
    args = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_-]+', args.route_name):
        parser.error('route_name must contain only letters, digits, underscores or hyphens')
    numeric = args.start is not None and args.goal is not None
    semantic = args.source is not None and args.target is not None
    if numeric == semantic or (args.start is not None) != (args.goal is not None) or (args.source is not None) != (args.target is not None):
        parser.error('provide --start/--goal OR --from-object/--to-object')
    if numeric:
        start, goal = ({'x': values[0], 'y': values[1], 'yaw': values[2],
                        'frame_id': args.frame_id} for values in (args.start, args.goal))
    else:
        start = semantic_pose(args.semantic_map, args.source)
        goal = semantic_pose(args.semantic_map, args.target)
    if start.get('frame_id', 'map') != goal.get('frame_id', 'map'):
        parser.error('start and goal must use the same frame')

    rclpy.init()
    node = rclpy.create_node('save_nav_path')
    node.set_parameters([Parameter('use_sim_time', value=True)])
    client = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    try:
        if not client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('/compute_path_to_pose is unavailable')
        request = ComputePathToPose.Goal()
        request.start = stamped_pose(node, start)
        request.goal = stamped_pose(node, goal)
        request.use_start = True
        sent = client.send_goal_async(request)
        rclpy.spin_until_future_complete(node, sent)
        handle = sent.result()
        if not handle.accepted:
            raise RuntimeError('Nav2 rejected the planning goal')
        finished = handle.get_result_async()
        rclpy.spin_until_future_complete(node, finished)
        response = finished.result()
        if response.status != GoalStatus.STATUS_SUCCEEDED or response.result.error_code != ComputePathToPose.Result.NONE:
            raise RuntimeError(f'Nav2 planning failed: {response.result.error_msg}')
        path = response.result.path
        if len(path.poses) < 2:
            raise RuntimeError('Nav2 returned an empty or one-pose path')
        output = ROOT / 'config/paths' / f'{args.route_name}.yaml'
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(yaml.safe_dump(path_to_dict(path), sort_keys=False))
        print(f'Saved {len(path.poses)} poses to {output}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
