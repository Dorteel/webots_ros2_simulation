"""Approximate camera geometry for execution correspondence, not perception."""
from math import atan2, degrees, sqrt, tan, isfinite

DISTANCE_AMBIGUITY_TOLERANCE_M = 0.05


def optical_position(point, geometry, scene_to_map):
    translation, matrix = scene_to_map
    mapped = [translation[r] + sum(matrix[r*3+c]*point[c] for c in range(3)) for r in range(3)]
    position, rotation, _, _ = geometry
    delta = [a-b for a,b in zip(mapped,position)]
    return [sum(rotation[r*3+c]*delta[r] for r in range(3)) for c in range(3)]


def camera_measurement(point, geometry, scene_to_map):
    """Frustum membership of an object origin, not an occlusion test."""
    x, y, depth = optical_position(point, geometry, scene_to_map)
    horizontal, vertical = atan2(x, depth), atan2(y, depth)
    return dict(depth=depth, distance=sqrt(x*x + y*y + depth*depth),
                horizontal_angle=degrees(horizontal), vertical_angle=degrees(vertical),
                visible=depth > 0 and abs(horizontal) <= geometry[2]/2
                and abs(vertical) <= geometry[3]/2)



def _dot(a, b):
    return sum(x*y for x,y in zip(a,b))


def _subtract(a, b):
    return [x-y for x,y in zip(a,b)]


def _cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def _clip_polygon(polygon, normal, offset=0.0):
    """Clip a convex face to dot(normal, point) >= offset."""
    result = []
    for a,b in zip(polygon, polygon[1:]+polygon[:1]):
        da, db = _dot(normal,a)-offset, _dot(normal,b)-offset
        if da >= 0:
            result.append(a)
        if (da >= 0) != (db >= 0):
            fraction = da/(da-db)
            result.append([x+fraction*(y-x) for x,y in zip(a,b)])
    return result


def _nearest_on_polygon(polygon):
    # Project onto the face plane when the projection is inside; otherwise
    # the closest point lies on an edge. This is not just corner sampling.
    candidates = list(polygon)
    if len(polygon) >= 3:
        normals = [_cross(_subtract(a,polygon[0]), _subtract(b,polygon[0]))
                   for a,b in zip(polygon[1:],polygon[2:])]
        normal = max(normals,key=lambda n:_dot(n,n))
        norm2 = _dot(normal,normal)
        if norm2 > 1e-20:
            projected = [v*_dot(normal,polygon[0])/norm2 for v in normal]
            signs = [_dot(normal,_cross(_subtract(b,a),_subtract(projected,a)))
                     for a,b in zip(polygon,polygon[1:]+polygon[:1])]
            if all(v >= -1e-12 for v in signs) or all(v <= 1e-12 for v in signs):
                candidates.append(projected)
    for a,b in zip(polygon,polygon[1:]+polygon[:1]):
        edge = _subtract(b,a)
        norm2 = _dot(edge,edge)
        if norm2 > 0:
            fraction = max(0.0,min(1.0,-_dot(a,edge)/norm2))
            candidates.append([x+fraction*v for x,v in zip(a,edge)])
    return min(candidates,key=lambda p:_dot(p,p))


def visible_bounds_surface(bounds, position, geometry, scene_to_map):
    """Nearest point on a world AABB's surface intersecting the optical frustum.

    Bounds are conservative collision extents relative to the object origin;
    the supervisor extractor has already applied the object's world rotation.
    No occlusion claim is made, and no synthetic object dimensions are used.
    """
    low, high = bounds
    corners = [optical_position([position[i]+(high[i] if bits & (1<<i) else low[i])
                                 for i in range(3)],geometry,scene_to_map) for bits in range(8)]
    faces = ((0,2,6,4),(1,5,7,3),(0,4,5,1),(2,3,7,6),(0,1,3,2),(4,6,7,5))
    h,v = tan(geometry[2]/2),tan(geometry[3]/2)
    planes = (((0,0,1),1e-6),((1,0,h),0),((-1,0,h),0),((0,1,v),0),((0,-1,v),0))
    nearest = []
    for face in faces:
        polygon = [corners[i] for i in face]
        for normal,offset in planes:
            polygon = _clip_polygon(polygon,normal,offset)
            if not polygon:
                break
        if polygon:
            nearest.append(_nearest_on_polygon(polygon))
    return min(nearest,key=lambda p:_dot(p,p)) if nearest else None


def object_camera_measurement(node, geometry, scene_to_map):
    if __package__:
        from .world_utils import _collision_bounds
    else:
        from world_utils import _collision_bounds
    position = node.getPosition()
    measurement = camera_measurement(position,geometry,scene_to_map)
    measurement['origin_visible'] = measurement['visible']
    measurement['visible'] = False
    measurement['nearest_visible_point'] = None
    try:
        bounds = _collision_bounds(node)
        if bounds is not None and (not all(isfinite(v) for side in bounds for v in side)
                                   or any(a>b for a,b in zip(*bounds))):
            bounds = None
    except (AttributeError, TypeError, ValueError, RuntimeError):
        bounds = None
    measurement['bounds_available'] = bounds is not None
    if bounds is not None:
        point = visible_bounds_surface(bounds,position,geometry,scene_to_map)
        measurement['nearest_visible_point'] = point
        measurement['visible'] = point is not None
        if point is not None:
            measurement['distance'] = sqrt(_dot(point,point))
    return measurement

def measurement_diagnostic(identifier, measurement):
    return (f"{identifier}: depth={measurement['depth']:.2f}m distance={measurement['distance']:.2f}m "
            f"horizontal_angle={measurement['horizontal_angle']:.1f}deg "
            f"vertical_angle={measurement['vertical_angle']:.1f}deg "
            f"visible={'yes' if measurement['visible'] else 'no'}"
            + (f" origin_visible={'yes' if measurement['origin_visible'] else 'no'}"
               f" bounding_volume_visible={'yes' if measurement['visible'] else 'no'}"
               f" bounds_available={'yes' if measurement['bounds_available'] else 'no'}"
               + (f" nearest_visible_distance={measurement['distance']:.3f}m" if measurement['visible'] else '')
               if 'origin_visible' in measurement else ''))


def select_camera_candidate(matches, geometry, scene_to_map=([0,0,0], [1,0,0,0,1,0,0,0,1])):
    """Choose the closest compatible bounding surface inside the optical frustum."""
    visible, diagnostics = [], []
    for identifier, node in matches:
        measurement = object_camera_measurement(node, geometry, scene_to_map)
        diagnostics.append(measurement_diagnostic(identifier, measurement) + ' semantic_compatible=yes')
        if measurement['visible']:
            visible.append((measurement['distance'], identifier, node))
    visible.sort(key=lambda item: item[0])
    if not visible:
        raise ValueError("camera grounding: no visible compatible candidate; no candidates inside FOV; "
                         + "; ".join(diagnostics))
    diagnostics.append('[GROUNDING] Visible compatible candidates:')
    diagnostics.extend(f"[GROUNDING]   {identifier}: distance={distance:.2f}m"
                       for distance, identifier, _ in visible)
    if len(visible) > 1 and visible[1][0] - visible[0][0] <= DISTANCE_AMBIGUITY_TOLERANCE_M:
        raise ValueError("camera grounding ambiguous between " + repr([v[1] for v in visible])
                         + f"; distance advantage={visible[1][0] - visible[0][0]:.3f}m"
                         + f"; required advantage>{DISTANCE_AMBIGUITY_TOLERANCE_M:.3f}m"
                         + "; " + "; ".join(diagnostics))
    diagnostics.append(f'[GROUNDING] Selected closest visible instance: {visible[0][1]}')
    return [(visible[0][1], visible[0][2])], diagnostics
