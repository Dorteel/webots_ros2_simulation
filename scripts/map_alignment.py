"""ROS-independent SE(2) alignment. All angles are radians; distances are metres."""
from dataclasses import dataclass
from datetime import datetime, timezone
import math
from pathlib import Path
import tempfile

import yaml


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def validate_convention(convention):
    # The existing odometry publisher only implements this world convention.
    # Reject other worlds instead of silently treating X/Z as X/Y.
    if not (convention.get('supported') is True and
            convention.get('webots_coordinate_system') == 'ENU' and
            convention.get('scene_ground_axes') == ['x', 'y'] and
            convention.get('ros_ground_axes') == ['x', 'y'] and
            convention.get('up_axis') == 'z' and
            convention.get('scene_to_odom') == 'identity_xy_yaw_about_z'):
        raise ValueError('Unsupported scene/Webots/odom convention; requires verified ENU X/Y')


@dataclass(frozen=True)
class MapAlignment:
    x: float
    y: float
    yaw: float

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.x, self.y, self.yaw)):
            raise ValueError('Alignment must contain finite values')

    @classmethod
    def from_poses(cls, scene, map_pose):
        """T_map_scene = T_map_base * inverse(T_scene_base), for [x,y,yaw]."""
        if len(scene) != 3 or len(map_pose) != 3 or not all(map(math.isfinite, (*scene, *map_pose))):
            raise ValueError('Poses must be finite [x, y, yaw] triples')
        yaw = wrap(map_pose[2] - scene[2])
        c, s = math.cos(yaw), math.sin(yaw)
        return cls(map_pose[0] - c * scene[0] + s * scene[1],
                   map_pose[1] - s * scene[0] - c * scene[1], yaw)

    def scene_to_map(self, x_scene, y_scene):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.x + c*x_scene - s*y_scene, self.y + s*x_scene + c*y_scene

    def map_to_scene(self, x_map, y_map):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        x, y = x_map - self.x, y_map - self.y
        return c*x + s*y, -s*x + c*y

    def raw_scene_to_map(self, xyz):
        """Generator qualities.location is raw world [X,Y,Z]; ENU drops Z."""
        if len(xyz) != 3:
            raise ValueError('Expected raw scene [X,Y,Z]')
        return self.scene_to_map(xyz[0], xyz[1])

    def errors(self, scene, map_pose):
        x, y = self.scene_to_map(*scene[:2])
        return math.hypot(x-map_pose[0], y-map_pose[1]), abs(wrap(scene[2]+self.yaw-map_pose[2]))

    @property
    def matrix(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return [[c, -s, self.x], [s, c, self.y], [0.0, 0.0, 1.0]]

    @classmethod
    def load(cls, path):
        data = yaml.safe_load(Path(path).read_text())
        if data.get('source_frame') != 'scene_graph' or data.get('target_frame') != 'map':
            raise ValueError('Expected scene_graph -> map alignment')
        validate_convention(data['coordinate_convention'])
        t = data['transform']
        alignment = cls(float(t['translation']['x']), float(t['translation']['y']), float(t['yaw']))
        if 'matrix' in data:
            matrix = data['matrix']
            if len(matrix) != 3 or any(len(row) != 3 for row in matrix):
                raise ValueError('Expected a 3x3 homogeneous matrix')
            if any(not math.isfinite(float(matrix[r][c])) or
                   abs(float(matrix[r][c]) - alignment.matrix[r][c]) > 1e-9
                   for r in range(3) for c in range(3)):
                raise ValueError('Matrix and transform disagree')
        return alignment


def unique_output_directory(root):
    root = Path(root)
    root.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')
    return Path(tempfile.mkdtemp(prefix=f'apartment_{stamp}_', dir=root))


class CompletionGate:
    """Ignore false/pause/start messages and repeated latched completion delivery."""
    def __init__(self):
        self.started = False

    def accept(self, complete):
        if not complete or self.started:
            return False
        self.started = True
        return True
