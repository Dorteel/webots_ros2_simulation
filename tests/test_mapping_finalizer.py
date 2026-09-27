"""Actual finalizer, deterministic TF/odom and a mock Nav2 SaveMap service."""
import math
from pathlib import Path
import sys
import tempfile
import time
import unittest

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from nav2_msgs.srv import SaveMap
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tf2_msgs.msg import TFMessage
import json
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from mapping_finalizer import MappingFinalizer
from map_alignment import MapAlignment
from test_map_alignment import CONVENTION


class FinalizerTests(unittest.TestCase):
    def exercise(self, mode):
        with tempfile.TemporaryDirectory() as root:
            output = Path(root)/'maps'
            output.mkdir()
            expected = MapAlignment(1.23, -0.42, 0.31)
            scene_path = Path(root)/'scene.json'
            scene_path.write_text(json.dumps({'objects': [
                dict(id='inside', type='Location', qualities=dict(location=[*expected.map_to_scene(0.025, 0.025), 99.])),
                dict(id='outside', type='Location', qualities=dict(location=[*expected.map_to_scene(10, 10), 99.])),
            ]}))
            rclpy.init(args=['--ros-args', '-p', f'maps_directory:={output}',
                            '-p', f'scene_graph:={scene_path}', '-p', 'finalization_timeout:=8.0'])
            source = Node('fallback_ground_truth_odom')
            finalizer = MappingFinalizer()
            executor = SingleThreadedExecutor()
            executor.add_node(source)
            executor.add_node(finalizer)
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            complete = source.create_publisher(Bool, '/exploration_complete', qos)
            convention = source.create_publisher(String, '/mapping/coordinate_convention', qos)
            maps = source.create_publisher(OccupancyGrid, '/map', qos)
            odom = source.create_publisher(Odometry, '/odom', 10)
            dynamic, static = TransformBroadcaster(source), StaticTransformBroadcaster(source)
            def map_tf(offset=0.0):
                tf = TransformStamped()
                tf.header.frame_id = 'map'
                tf.child_frame_id = 'odom'
                tf.transform.translation.x = expected.x + offset
                tf.transform.translation.y = expected.y
                tf.transform.rotation.z = math.sin(expected.yaw/2)
                tf.transform.rotation.w = math.cos(expected.yaw/2)
                static.pub_tf.publish(TFMessage(transforms=[tf]))
            map_tf()
            def tick():
                m = Odometry()
                m.header.stamp = source.get_clock().now().to_msg()
                m.header.frame_id = 'odom'
                m.child_frame_id = 'base_link'
                m.pose.pose.position.x, m.pose.pose.position.y = 2.0, -1.0
                m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(0.2), math.cos(0.2)
                odom.publish(m)
                tf = TransformStamped()
                tf.header, tf.child_frame_id = m.header, m.child_frame_id
                tf.transform.translation.x, tf.transform.translation.y = 2.0, -1.0
                tf.transform.rotation = m.pose.pose.orientation
                if mode != 'missing_tf':
                    dynamic.sendTransform(tf)
            timer = source.create_timer(0.05, tick)
            convention.publish(String(data=json.dumps(dict(**CONVENTION, odom_source=source.get_name()))))
            grid = OccupancyGrid()
            grid.header.frame_id = 'map'
            grid.info.width = grid.info.height = 2
            grid.info.resolution = 0.05
            grid.data = [0]*4
            maps.publish(grid)
            calls = []
            def save(request, response):
                calls.append(request)
                self.assertEqual(request.image_format, 'pgm')
                if mode == 'save_failure':
                    response.result = False
                    return response
                stem = Path(request.map_url)
                stem.with_suffix('.pgm').write_bytes(b'P5\n2 2\n255\n'+b'\xff'*4)
                stem.with_suffix('.yaml').write_text(yaml.safe_dump(dict(image='map.pgm', resolution=0.05, origin=[0.,0.,0.])))
                if mode == 'drift':
                    map_tf(0.2)
                response.result = True
                return response
            service = source.create_service(SaveMap, '/mapping_map_saver/save_map', save)
            def spin_until(predicate, seconds):
                deadline = time.monotonic()+seconds
                while not predicate() and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=0.05)
            try:
                complete.publish(Bool(data=False))
                spin_until(lambda: False, 0.5)
                self.assertEqual(calls, [])
                self.assertEqual(list(output.iterdir()), [])
                complete.publish(Bool(data=True))
                spin_until(lambda: finalizer.state in ('done', 'failed'), 9)
                if mode == 'missing_tf':
                    self.assertEqual(finalizer.state, 'failed')
                    self.assertEqual(calls, [])
                    self.assertEqual(list(output.iterdir()), [])
                    return
                self.assertEqual(len(calls), 1)
                directory = next(output.iterdir())
                if mode == 'success':
                    self.assertEqual(finalizer.state, 'done')
                    a = MapAlignment.load(directory/'alignment.yaml')
                    self.assertAlmostEqual(a.x, expected.x)
                    self.assertAlmostEqual(a.y, expected.y)
                    self.assertAlmostEqual(a.yaw, expected.yaw)
                    metadata = yaml.safe_load((directory/'alignment.yaml').read_text())
                    self.assertEqual(len(metadata['validation']['samples']), 3)
                    self.assertEqual([r['inside_bounds'] for r in metadata['room_bounds']['rooms']], [True, False])
                    self.assertGreater(metadata['validation']['samples'][0]['stamp'], metadata['calibration_stamp'])
                else:
                    self.assertEqual(finalizer.state, 'failed')
                    self.assertFalse((directory/'alignment.yaml').exists())
                    self.assertTrue((directory/'FINALIZATION_FAILED.txt').exists())
                complete.publish(Bool(data=True))
                spin_until(lambda: False, 0.3)
                self.assertEqual(len(calls), 1)
            finally:
                executor.shutdown()
                finalizer.destroy_node()
                source.destroy_node()
                rclpy.shutdown()

    def test_completion_saves_once_and_validates_later_samples(self):
        self.exercise('success')

    def test_save_failure_never_writes_alignment(self):
        self.exercise('save_failure')

    def test_missing_tf_times_out_without_saving(self):
        self.exercise('missing_tf')

    def test_changed_alignment_is_rejected(self):
        self.exercise('drift')


if __name__ == '__main__':
    unittest.main()
