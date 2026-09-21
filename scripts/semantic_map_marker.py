#!/usr/bin/env python3
"""Save an approach pose and object point from two RViz Publish Point clicks."""

import argparse
from math import atan2
from pathlib import Path

from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.node import Node
import yaml

MAP_FILE = Path(__file__).resolve().parents[1] / 'config' / 'semantic_navigation_map.yaml'


class ClickCollector(Node):
    def __init__(self):
        super().__init__('semantic_map_marker')
        self.name = ''
        self.points = []
        self.create_subscription(PointStamped, '/clicked_point', self.on_click, 10)

    def on_click(self, message):
        self.points.append(message)
        if len(self.points) == 1:
            print(f'Click OBJECT position for {self.name}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('name', nargs='?', help='optional first object name')
    first_name = parser.parse_args().name

    # Load once; each completed object is written immediately.
    try:
        objects = yaml.safe_load(MAP_FILE.read_text()) if MAP_FILE.exists() else {}
    except yaml.YAMLError as exc:
        parser.error(f'cannot read {MAP_FILE}: {exc}')
    if objects is None:
        objects = {}
    if not isinstance(objects, dict):
        parser.error(f'{MAP_FILE} must contain a YAML mapping')

    rclpy.init()
    node = ClickCollector()
    try:
        while True:
            try:
                name = (first_name or input('\nObject name: ')).strip()
            except EOFError:
                break
            first_name = None
            if name.lower() in ('q', 'quit', 'exit'):
                break
            if not name:
                continue
            if name in objects and input(f'{name} already exists. Overwrite? [y/N] ').strip().lower() != 'y':
                print('Not saved')
                continue

            node.name = name
            node.points.clear()
            print(f'Click APPROACH position for {name}', flush=True)
            while len(node.points) < 2:
                rclpy.spin_once(node)

            approach, target = node.points
            frame = approach.header.frame_id
            if not frame or frame != target.header.frame_id:
                print('Not saved: both clicks must have the same non-empty frame_id')
                continue

            # Face the object from the selected approach position.
            entry = dict(objects.get(name, {}))
            entry.update({
                'navigation_pose_source': 'manual',
                'navigation_pose': {
                    'frame_id': frame,
                    'x': float(approach.point.x),
                    'y': float(approach.point.y),
                    'yaw': atan2(target.point.y - approach.point.y,
                                 target.point.x - approach.point.x),
                },
                'object_position': {
                    'x': float(target.point.x),
                    'y': float(target.point.y),
                },
            })
            entry['preferred_navigation_pose'] = entry['navigation_pose']
            objects[name] = entry
            MAP_FILE.write_text(yaml.safe_dump(objects, sort_keys=False, allow_unicode=True))
            print(f'Saved {name}')
    except KeyboardInterrupt:
        print('\nStopped semantic mapping')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
