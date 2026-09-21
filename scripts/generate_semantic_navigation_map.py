#!/usr/bin/env python3
"""Build map-frame object and approach poses from the apartment Webots world."""

from math import atan2, cos, pi, sin
from pathlib import Path

import numpy as np
from PIL import Image
import yaml

from extract_semantic_map import extract, slugify

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / 'config/semantic_navigation_map.yaml'


def map_grid(path):
    info = yaml.safe_load(path.read_text())
    pixels = np.array(Image.open(path.parent / info['image']))
    # Nav2 trinary map: white is free, black occupied, gray unknown.
    free = pixels >= round(255 * (1 - info['free_thresh']))
    return free, float(info['resolution']), info['origin']


def clearance(free, resolution, origin, x, y, radius):
    """Require every cell under a circular robot footprint to be known free."""
    ox, oy, angle = origin
    dx, dy = x - ox, y - oy
    mx = (cos(angle) * dx + sin(angle) * dy) / resolution
    my = (-sin(angle) * dx + cos(angle) * dy) / resolution
    if mx < 0 or my < 0 or mx >= free.shape[1] or my >= free.shape[0]:
        return False
    cx, cy = int(mx), free.shape[0] - 1 - int(my)
    cells = int(np.ceil(radius / resolution))
    for row in range(cy - cells, cy + cells + 1):
        for col in range(cx - cells, cx + cells + 1):
            if (col - cx)**2 + (row - cy)**2 > (radius / resolution)**2:
                continue
            if row < 0 or col < 0 or row >= free.shape[0] or col >= free.shape[1] or not free[row, col]:
                return False
    return True


def map_pose(webots_pose, transform):
    yaw = float(transform['yaw_offset'])
    x, y = webots_pose['x'], webots_pose['y']
    return {'frame_id': 'map',
            'x': round(float(transform['translation_x']) + cos(yaw)*x - sin(yaw)*y, 6),
            'y': round(float(transform['translation_y']) + sin(yaw)*x + cos(yaw)*y, 6),
            'yaw': round(atan2(sin(webots_pose['yaw'] + yaw), cos(webots_pose['yaw'] + yaw)), 6)}


def candidates(pose, settings, grid):
    free, resolution, origin = grid
    found = []
    for radius in settings['approach_radii']:
        for index in range(settings['candidate_count']):
            angle = 2*pi*index/settings['candidate_count']
            x, y = pose['x'] + radius*cos(angle), pose['y'] + radius*sin(angle)
            if not clearance(free, resolution, origin, x, y, settings['robot_clearance']):
                continue
            found.append({'frame_id': 'map', 'x': round(x, 6), 'y': round(y, 6),
                          'yaw': round(atan2(pose['y']-y, pose['x']-x), 6)})
    # Prefer positions closest to the object; preserve all valid alternatives.
    return found


def main():
    settings = yaml.safe_load((ROOT / 'config/semantic_map_generation.yaml').read_text())
    world = extract(ROOT / 'worlds/complete_apartment_tiago_ros2.wbt')['objects']
    grid = map_grid(ROOT / 'maps/kitchen.yaml')
    existing = yaml.safe_load(OUTPUT.read_text()) if OUTPUT.exists() else {}
    if existing is None:
        existing = {}
    if not isinstance(existing, dict):
        raise ValueError(f'{OUTPUT} must contain a YAML mapping')
    keys_by_slug = {slugify(key): key for key in existing}
    for entry in existing.values():
        if isinstance(entry, dict) and entry.get('navigation_pose') is not None:
            entry.setdefault('navigation_pose_source', 'manual')
            entry.setdefault('preferred_navigation_pose', entry['navigation_pose'])
    anchors = generated = 0
    for object_id, source in world.items():
        key = keys_by_slug.get(object_id, object_id)
        old = existing.get(key, {})
        if not isinstance(old, dict):
            old = {}
        pose = map_pose(source['pose'], settings['webots_to_map'])
        entry = dict(old)
        entry.update(type=source['type'], webots_name=source['webots_name'],
                     webots_pose=source['pose'], object_pose=pose)
        is_anchor = (source['type'] in settings['anchor_types'] or
                     any(object_id.startswith(prefix) for prefix in settings['anchor_name_prefixes']))
        if is_anchor:
            anchors += 1
            # Legacy clicked entries have a navigation_pose but no source label.
            manual = old.get('navigation_pose_source') == 'manual' or (
                'navigation_pose_source' not in old and old.get('navigation_pose') is not None)
            if manual:
                entry['navigation_pose_source'] = 'manual'
                if old.get('navigation_pose') is not None:
                    entry.setdefault('preferred_navigation_pose', old['navigation_pose'])
            else:
                poses = candidates(pose, settings, grid)
                entry['navigation_poses'] = poses
                entry['preferred_navigation_pose'] = poses[0] if poses else None
                entry['navigation_pose'] = poses[0] if poses else None
                entry['navigation_pose_source'] = 'automatic' if poses else 'needs_manual'
                generated += bool(poses)
        existing[key] = entry
    OUTPUT.write_text(yaml.safe_dump(existing, sort_keys=False, allow_unicode=True))
    print(f'Found {len(world)} semantic objects')
    print(f'Found {anchors} navigation anchors')
    print(f'Generated navigation poses for {generated}')
    print(f'{sum(entry.get("navigation_pose_source") == "needs_manual" for entry in existing.values() if isinstance(entry, dict))} require manual annotation')
    print(f'Saved {OUTPUT}')


if __name__ == '__main__':
    main()
