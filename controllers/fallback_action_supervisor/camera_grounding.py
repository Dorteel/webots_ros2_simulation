"""Approximate camera geometry for execution correspondence, not perception."""
from math import atan2, degrees, hypot, pi, sqrt

MIN_ANGULAR_MARGIN = 5 * pi / 180


def optical_position(point, geometry, scene_to_map):
    translation, matrix = scene_to_map
    mapped = [translation[r] + sum(matrix[r*3+c]*point[c] for c in range(3)) for r in range(3)]
    position, rotation, _, _ = geometry
    delta = [a-b for a,b in zip(mapped,position)]
    return [sum(rotation[r*3+c]*delta[r] for r in range(3)) for c in range(3)]


def select_camera_candidate(matches, geometry, scene_to_map=([0,0,0], [1,0,0,0,1,0,0,0,1])):
    """Project map poses into ROS optical coordinates: +Z forward, +Y down."""
    position, rotation, horizontal_fov, vertical_fov = geometry
    visible, diagnostics = [], []
    for identifier, node in matches:
        x, y, z = optical_position(node.getPosition(), geometry, scene_to_map)
        depth = z
        distance = sqrt(x*x + y*y + z*z)
        horizontal, vertical = atan2(x, depth), atan2(y, depth)
        angular = atan2(hypot(x, y), depth)
        inside = depth > 0 and abs(horizontal) <= horizontal_fov / 2 and abs(vertical) <= vertical_fov / 2
        diagnostics.append(
            f"{identifier}: depth={depth:.2f}m distance={distance:.2f}m "
            f"horizontal_angle={degrees(horizontal):.1f}deg vertical_angle={degrees(vertical):.1f}deg "
            f"visible={'yes' if inside else 'no'}")
        if inside:
            visible.append((angular, distance, identifier, node))
    visible.sort(key=lambda item: (item[0], item[1]))
    if not visible:
        raise ValueError("camera grounding: no candidates inside FOV; " + "; ".join(diagnostics))
    if len(visible) > 1 and visible[1][0] - visible[0][0] < MIN_ANGULAR_MARGIN:
        raise ValueError("camera grounding ambiguous between " + repr([v[2] for v in visible])
                         + f"; angular advantage={degrees(visible[1][0] - visible[0][0]):.2f}deg"
                         + f"; required advantage={degrees(MIN_ANGULAR_MARGIN):.2f}deg"
                         + "; " + "; ".join(diagnostics))
    return [(visible[0][2], visible[0][3])], diagnostics
