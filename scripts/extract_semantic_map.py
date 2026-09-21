#!/usr/bin/env python3
"""Extract semantic object poses from a Webots .wbt world file.

The script resolves nested Webots transforms (e.g. objects inside KITCHEN_BLOCK)
and writes each static scene entity's world-frame position and 2D yaw.

Important:
- These are OBJECT poses, not safe Nav2 approach poses.
- `navigation_pose` is intentionally left null for later annotation.
"""

from pathlib import Path
import argparse
import math
import re

import numpy as np
import yaml


NODE_RE = re.compile(
    r"^\s*(?:DEF\s+(\S+)\s+)?([A-Za-z_][A-Za-z0-9_+]*)\s*\{\s*$"
)

# Appearance/device PROTOs are not semantic scene objects.
EXCLUDED_TYPES = {
    "TexturedBackground",
    "Roughcast",
    "Parquetry",
    "Pavement",
    "PorcelainChevronTiles",
    "CementTiles",
    "CabinetHandle",
    "PaintedWood",
    "CarpetFibers",
    "BrushedAluminium",
    "BlanketFabric",
    "VarnishedPine",
    "HokuyoUrg04lxug01",
    "TiagoGripperConnector",
    # Dynamic/agent entities are excluded from the static semantic map.
    "Tiago++",
    "Pedestrian",
}


class Node:
    """Minimal representation of a Webots node needed for pose extraction."""

    def __init__(self, node_type, def_name=None, parent=None, line=0, depth=0):
        self.node_type = node_type
        self.def_name = def_name
        self.parent = parent
        self.line = line
        self.depth = depth
        self.translation = np.zeros(3)
        self.rotation = np.array([0.0, 0.0, 1.0, 0.0])
        self.name = None


def axis_angle_to_matrix(rotation):
    """Convert Webots axis-angle rotation [x, y, z, angle] to a 3x3 matrix."""
    x, y, z, angle = rotation
    norm = math.sqrt(x * x + y * y + z * z)
    if norm == 0.0 or abs(angle) < 1e-15:
        return np.eye(3)

    x, y, z = x / norm, y / norm, z / norm
    c, s, C = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)

    return np.array(
        [
            [c + x*x*C,     x*y*C - z*s, x*z*C + y*s],
            [y*x*C + z*s,   c + y*y*C,   y*z*C - x*s],
            [z*x*C - y*s,   z*y*C + x*s, c + z*z*C],
        ]
    )


def parse_world(path):
    """Parse node nesting and direct translation/rotation/name fields."""
    lines = path.read_text().splitlines()
    nodes, stack = [], []
    brace_depth = 0

    for line_number, line in enumerate(lines, start=1):
        match = NODE_RE.match(line)

        if match:
            def_name, node_type = match.groups()
            parent = stack[-1] if stack else None
            node = Node(node_type, def_name, parent, line_number, brace_depth)
            nodes.append(node)
            stack.append(node)
            brace_depth += line.count("{") - line.count("}")
            continue

        if stack:
            node = stack[-1]

            # Only read fields belonging directly to this node, not its children.
            if brace_depth == node.depth + 1:
                translation = re.match(r"^\s*translation\s+(.+)", line)
                if translation:
                    values = translation.group(1).split()
                    if len(values) >= 3:
                        node.translation = np.array(
                            [float(v) for v in values[:3]]
                        )

                rotation = re.match(r"^\s*rotation\s+(.+)", line)
                if rotation:
                    values = rotation.group(1).split()
                    if len(values) >= 4:
                        node.rotation = np.array(
                            [float(v) for v in values[:4]]
                        )

                name = re.match(r'^\s*name\s+"(.*)"', line)
                if name:
                    node.name = name.group(1)

        brace_depth += line.count("{") - line.count("}")

        while stack and brace_depth <= stack[-1].depth:
            stack.pop()

    # Discover EXTERNPROTO types from this world itself.
    extern_types = set()
    for line in lines:
        match = re.match(r'EXTERNPROTO\s+"[^"]*/([^/]+)\.proto"', line)
        if match:
            extern_types.add(match.group(1).replace("%2B", "+"))

    return nodes, extern_types


def world_transform(node, cache):
    """Recursively compose local transforms into Webots world coordinates."""
    if id(node) in cache:
        return cache[id(node)]

    transform = np.eye(4)
    transform[:3, :3] = axis_angle_to_matrix(node.rotation)
    transform[:3, 3] = node.translation

    if node.parent is not None:
        transform = world_transform(node.parent, cache) @ transform

    cache[id(node)] = transform
    return transform


def slugify(value):
    """Create a stable YAML key from a Webots name."""
    value = value.lower().strip()
    value = re.sub(r"\((\d+)\)", r"_\1", value)
    value = re.sub(r"[^a-z0-9]+", "_", value)
    return value.strip("_")


def semantic_type(node_type):
    """Remove the implementation-specific Connector suffix when present."""
    return node_type.removesuffix("Connector")


def extract(path):
    nodes, extern_types = parse_world(path)

    # Match the semantic scene entities in this world:
    # external object PROTO instances + explicitly defined Solid objects.
    selected = [
        node for node in nodes
        if (
            (node.node_type in extern_types and node.node_type not in EXCLUDED_TYPES)
            or node.node_type == "Solid"
        )
    ]

    type_counts = {}
    used_ids = set()
    objects = {}
    transform_cache = {}

    for node in selected:
        type_counts[node.node_type] = type_counts.get(node.node_type, 0) + 1

        if node.name:
            object_id = slugify(node.name)
        elif node.def_name:
            object_id = slugify(node.def_name)
        else:
            base = slugify(semantic_type(node.node_type))
            object_id = f"{base}_{type_counts[node.node_type]}"

        # Guarantee uniqueness even if Webots names collide.
        original_id = object_id
        duplicate = 2
        while object_id in used_ids:
            object_id = f"{original_id}_{duplicate}"
            duplicate += 1
        used_ids.add(object_id)

        transform = world_transform(node, transform_cache)
        rotation = transform[:3, :3]
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        x, y, z = transform[:3, 3]

        objects[object_id] = {
            "type": semantic_type(node.node_type),
            "webots_type": node.node_type,
            "webots_name": node.name,
            "source_line": node.line,
            "pose": {
                "x": round(float(x), 6),
                "y": round(float(y), 6),
                "z": round(float(z), 6),
                "yaw": round(float(yaw), 6),
            },
            # To be annotated later with a collision-free Nav2 approach pose.
            "navigation_pose": None,
        }

    return {
        "metadata": {
            "source": path.name,
            "frame_id": "webots_world",
            "object_count": len(objects),
            "note": (
                "Object origins extracted from Webots. These are not automatically "
                "safe Nav2 approach poses; annotate navigation_pose separately."
            ),
        },
        "objects": objects,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("world", type=Path, help="Input Webots .wbt file")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=Path("semantic_object_poses.yaml"),
        help="Output YAML file",
    )
    args = parser.parse_args()

    data = extract(args.world)
    args.output.write_text(
        yaml.safe_dump(data, sort_keys=False, allow_unicode=True)
    )
    print(f"Wrote {data['metadata']['object_count']} objects to {args.output}")


if __name__ == "__main__":
    main()
