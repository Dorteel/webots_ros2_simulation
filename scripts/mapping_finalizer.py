#!/usr/bin/env python3
"""Save a completed map through Nav2 and validate its scene-graph alignment."""
from collections import deque
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import Odometry, OccupancyGrid
from nav2_msgs.srv import SaveMap
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener, TransformException
import yaml

from map_alignment import MapAlignment, CompletionGate, unique_output_directory, validate_convention


def yaw(q):
    values = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values)-1) > 0.01:
        raise ValueError('Invalid quaternion')
    if abs(q.x) > 0.025 or abs(q.y) > 0.025:
        raise ValueError('Nonplanar TF/odometry cannot establish a 2D alignment')
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class MappingFinalizer(Node):
    def __init__(self):
        super().__init__('mapping_finalizer')
        self.root = Path(self.declare_parameter('maps_directory', str(Path(__file__).resolve().parents[1]/'maps')).value)
        self.scene_graph = self.declare_parameter('scene_graph', '').value
        self.world = self.declare_parameter('world', '').value
        self.timeout = self.declare_parameter('finalization_timeout', 90.0).value
        self.position_tolerance = self.declare_parameter('position_tolerance', 0.05).value
        self.yaw_tolerance = self.declare_parameter('yaw_tolerance', 0.03).value
        if not all(math.isfinite(v) and v > 0 for v in (self.timeout, self.position_tolerance, self.yaw_tolerance)):
            raise ValueError('Finalization timeout and validation tolerances must be finite and positive')
        self.gate = CompletionGate()
        self.state = 'idle'
        self.directory = None
        self.convention = None
        self.grid = None
        self.odometry = deque(maxlen=100)
        self.odom_received = 0.0
        self.still_since = None
        self.samples = []
        self.last_validation_stamp = None
        self.buffer = Buffer(cache_time=Duration(seconds=30))
        self.listener = TransformListener(self.buffer, self)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/exploration_complete', self.complete, qos)
        self.create_subscription(String, '/mapping/coordinate_convention', self.conventions, qos)
        self.create_subscription(OccupancyGrid, '/map', self.map_received, qos)
        self.create_subscription(Odometry, '/odom', self.odom_received_callback, 50)
        self.saver = self.create_client(SaveMap, '/mapping_map_saver/save_map')
        # A wall/steady timer keeps the watchdog effective even if /clock stops.
        self.timer = self.create_timer(0.1, self.step, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Waiting for genuine exploration completion')

    def conventions(self, msg):
        try:
            self.convention = json.loads(msg.data)
        except ValueError:
            self.convention = {'supported': False}

    def map_received(self, msg):
        self.grid = msg

    def odom_received_callback(self, msg):
        self.odometry.append(msg)
        self.odom_received = time.monotonic()

    def complete(self, msg):
        if self.gate.accept(msg.data):
            self.state = 'settling'
            self.deadline = time.monotonic() + self.timeout
            self.get_logger().info('Exploration complete; waiting for stationary, synchronized poses')

    def sample(self):
        if self.convention is None or self.grid is None or not self.odometry:
            return None
        validate_convention(self.convention)
        publishers = self.get_publishers_info_by_topic('/odom')
        if len(publishers) != 1 or publishers[0].node_name != self.convention['odom_source']:
            raise ValueError('Odometry is not exclusively from the declared Supervisor')
        if self.grid.header.frame_id != 'map' or not self.grid.data:
            raise ValueError('A nonempty map-frame occupancy grid is required')
        if time.monotonic() - self.odom_received > 3:
            return None
        now = self.get_clock().now()
        for msg in reversed(self.odometry):
            stamp = Time.from_msg(msg.header.stamp)
            age = (now-stamp).nanoseconds/1e9
            if age < 0.15 or age > 1.0:
                continue
            if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
                raise ValueError('Unexpected ground-truth odometry frames')
            try:
                mapped = self.buffer.lookup_transform('map', 'base_link', stamp).transform
                odom_tf = self.buffer.lookup_transform('odom', 'base_link', stamp).transform
            except TransformException:
                continue
            p = msg.pose.pose.position
            scene = [p.x, p.y, yaw(msg.pose.pose.orientation)]
            mapped_pose = [mapped.translation.x, mapped.translation.y, yaw(mapped.rotation)]
            odom_pose = [odom_tf.translation.x, odom_tf.translation.y, yaw(odom_tf.rotation)]
            if not all(math.isfinite(v) for v in (*scene, *mapped_pose, *odom_pose)):
                raise ValueError('Nonfinite pose data')
            identity = MapAlignment(0, 0, 0)
            error, angle = identity.errors(scene, odom_pose)
            if error > 0.005 or angle > 0.005:
                raise ValueError('Odometry and odom->base_link TF disagree at the same timestamp')
            speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
            stationary = speed < 0.01 and abs(msg.twist.twist.angular.z) < 0.02
            return dict(stamp=stamp.nanoseconds/1e9, scene=scene, map=mapped_pose, stationary=stationary)
        return None

    def step(self):
        if self.state in ('idle', 'done', 'failed'):
            return
        try:
            if time.monotonic() > self.deadline:
                raise TimeoutError('Finalization timed out waiting for map saver, TF, or stable pose samples')
            if self.state == 'saving':
                if not self.save_future.done():
                    return
                response = self.save_future.result()
                if response is None or not response.result:
                    raise RuntimeError('Nav2 SaveMap service failed')
                if not all((self.directory/name).is_file() for name in ('map.yaml', 'map.pgm')):
                    raise RuntimeError('SaveMap returned success without expected map files')
                self.get_logger().info('Map saved; computing scene/map alignment...')
                self.alignment = MapAlignment.from_poses(self.calibration['scene'], self.calibration['map'])
                self.state = 'validating'
            sample = self.sample()
            if sample is None:
                return
            if self.state == 'settling':
                if not sample['stationary']:
                    self.still_since = None
                    return
                if self.still_since is None:
                    self.still_since = sample['stamp']
                if sample['stamp'] - self.still_since < 1.0 or not self.saver.service_is_ready():
                    return
                self.calibration = sample
                self.last_validation_stamp = sample['stamp']
                self.directory = unique_output_directory(self.root)
                request = SaveMap.Request()
                request.map_topic = '/map'
                request.map_url = str(self.directory/'map')
                request.image_format = 'pgm'
                request.map_mode = 'trinary'
                request.free_thresh = 0.25
                request.occupied_thresh = 0.65
                self.get_logger().info(f'Saving map... {self.directory}')
                self.save_future = self.saver.call_async(request)
                self.state = 'saving'
            elif self.state == 'validating':
                if not sample['stationary']:
                    raise ValueError('Robot moved during finalization; map/alignment snapshot is not reliable')
                if sample['stamp'] - self.last_validation_stamp < 0.2:
                    return
                position_error, yaw_error = self.alignment.errors(sample['scene'], sample['map'])
                if position_error > self.position_tolerance or yaw_error > self.yaw_tolerance:
                    raise ValueError(f'Alignment validation failed: {position_error:.6f} m, {yaw_error:.6f} rad')
                self.samples.append(dict(**sample, position_error_m=position_error, yaw_error_rad=yaw_error))
                self.last_validation_stamp = sample['stamp']
                if len(self.samples) == 3:
                    self.write_alignment()
                    self.state = 'done'
                    self.get_logger().info(f'Mapping finalized: {self.directory}')
        except Exception as error:
            self.state = 'failed'
            self.get_logger().error(f'Mapping finalization FAILED: {error}')
            if self.directory is not None:
                (self.directory/'FINALIZATION_FAILED.txt').write_text(str(error)+'\n')
            # Do not retry a consumed completion signal or advertise a partial map
            # as finalized. Keep the node alive so the launch log remains visible.

    def room_checks(self):
        if not self.scene_graph:
            return dict(status='not_requested', reason='No scene_graph argument supplied')
        data = json.loads(Path(self.scene_graph).read_text())
        # Read the SAVED map's dimensions and origin, not a possibly newer /map.
        metadata = yaml.safe_load((self.directory/'map.yaml').read_text())
        with (self.directory/'map.pgm').open('rb') as image:
            tokens = []
            while len(tokens) < 4:
                line = image.readline()
                if not line:
                    raise ValueError('Invalid saved PGM header')
                tokens.extend(line.split(b'#', 1)[0].split())
        width, height = int(tokens[1]), int(tokens[2])
        origin = MapAlignment(*map(float, metadata['origin']))
        results = []
        for obj in data['objects']:
            point = obj.get('qualities', {}).get('location')
            if obj.get('type') != 'Location' or point is None:
                continue
            mapped = self.alignment.raw_scene_to_map(point)
            gx, gy = origin.map_to_scene(*mapped)
            inside = 0 <= gx < width*metadata['resolution'] and 0 <= gy < height*metadata['resolution']
            results.append(dict(id=obj['id'], raw_scene_position=point, map_position=list(mapped), inside_bounds=inside))
        return dict(status='checked', scene_graph=str(Path(self.scene_graph).resolve()), rooms=results,
                    note='Bounds only; no free-cell or full-coverage assertion')

    def write_alignment(self):
        pos_error = max(s['position_error_m'] for s in self.samples)
        yaw_error = max(s['yaw_error_rad'] for s in self.samples)
        data = dict(source_frame='scene_graph', target_frame='map', generated_at=datetime.now(timezone.utc).isoformat(),
                    coordinate_convention=self.convention,
                    transform=dict(translation=dict(x=self.alignment.x, y=self.alignment.y), yaw=self.alignment.yaw),
                    matrix=self.alignment.matrix, world=self.world,
                    derivation='T_map_scene = T_map_base * inverse(T_scene_base); synchronized ROS timestamps',
                    robot_scene_pose=self.calibration['scene'], robot_map_pose=self.calibration['map'],
                    calibration_stamp=self.calibration['stamp'],
                    validation=dict(position_error_m=pos_error, yaw_error_rad=yaw_error,
                                    position_tolerance_m=self.position_tolerance, yaw_tolerance_rad=self.yaw_tolerance,
                                    samples=self.samples), room_bounds=self.room_checks(),
                    map_yaml='map.yaml', completion='no useful eligible frontiers; not a full-coverage guarantee')
        temporary = self.directory/'alignment.yaml.tmp'
        temporary.write_text(yaml.safe_dump(data, sort_keys=False))
        MapAlignment.load(temporary)  # Validate the serialized contract as well.
        temporary.rename(self.directory/'alignment.yaml')
        self.get_logger().info(f'Alignment validated: position error {pos_error:.6f} m; yaw error {yaw_error:.6f} rad (3 later samples)')


def main():
    rclpy.init()
    node = MappingFinalizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
