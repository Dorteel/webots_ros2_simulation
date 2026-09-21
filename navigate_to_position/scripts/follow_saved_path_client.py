#!/usr/bin/env python3
"""Load a saved Nav2 path and send it to /follow_path."""

import argparse
import math
from pathlib import Path

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path as NavPath
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
import yaml


def path_to_dict(path):
    """Keep every path pose, including its full orientation, in plain YAML."""
    return {
        'frame_id': path.header.frame_id,
        'poses': [
            {'position': [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z],
             'orientation': [pose.pose.orientation.x, pose.pose.orientation.y,
                             pose.pose.orientation.z, pose.pose.orientation.w]}
            for pose in path.poses
        ],
    }


def load_saved_path(filename):
    """Rebuild a Path; zero timestamps ask TF for the latest transform."""
    data = yaml.safe_load(Path(filename).read_text())
    if not isinstance(data, dict) or not isinstance(data.get('frame_id'), str) or not data['frame_id']:
        raise ValueError('saved path needs a non-empty frame_id')
    if not isinstance(data.get('poses'), list) or len(data['poses']) < 2:
        raise ValueError('saved path needs at least two poses')
    path = NavPath()
    path.header.frame_id = data['frame_id']
    for entry in data['poses']:
        position, orientation = entry['position'], entry['orientation']
        if len(position) != 3 or len(orientation) != 4:
            raise ValueError('each pose needs 3 position and 4 orientation values')
        values = [float(value) for value in (*position, *orientation)]
        if not all(math.isfinite(value) for value in values):
            raise ValueError('path coordinates must be finite')
        pose = PoseStamped()
        pose.header.frame_id = path.header.frame_id
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = values[:3]
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = values[3:]
        path.poses.append(pose)
    return path


class FollowSavedPathClient:
    def __init__(self, node):
        self._client = ActionClient(node, FollowPath, '/follow_path')

    async def follow(self, filename):
        """Return True only when Nav2 successfully follows the saved path."""
        path = load_saved_path(filename)
        if not self._client.wait_for_server(timeout_sec=5.0):
            return False
        handle = await self._client.send_goal_async(FollowPath.Goal(path=path))
        if not handle.accepted:
            return False
        response = await handle.get_result_async()
        return (response.status == GoalStatus.STATUS_SUCCEEDED
                and response.result.error_code == FollowPath.Result.NONE)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('path', type=Path, help='saved config/paths/*.yaml file')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('follow_saved_path_client')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        task = executor.create_task(FollowSavedPathClient(node).follow(args.path))
        executor.spin_until_future_complete(task)
        print('Path completed' if task.result() else 'Path failed')
        return 0 if task.result() else 1
    finally:
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
