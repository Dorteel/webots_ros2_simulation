"""Pure numerical/contract tests; no ROS imports or simulator required."""
import math
from pathlib import Path
import sys
import tempfile
import unittest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from map_alignment import MapAlignment, CompletionGate, unique_output_directory, validate_convention

CONVENTION = dict(supported=True, webots_coordinate_system='ENU', scene_ground_axes=['x', 'y'],
                  ros_ground_axes=['x', 'y'], up_axis='z', scene_to_odom='identity_xy_yaw_about_z')


class AlignmentTests(unittest.TestCase):
    def test_known_rotation_translation_and_inverse(self):
        a = MapAlignment(2, -3, math.pi/2)
        self.assertAlmostEqual(a.scene_to_map(4, 5)[0], -3)
        self.assertAlmostEqual(a.scene_to_map(4, 5)[1], 1)
        self.assertAlmostEqual(a.map_to_scene(-3, 1)[0], 4)
        self.assertAlmostEqual(a.map_to_scene(-3, 1)[1], 5)
        for x, y in [(-5, 7), (0, 0), (123.456, -89.1)]:
            xx, yy = a.map_to_scene(*a.scene_to_map(x, y))
            self.assertAlmostEqual(xx, x, places=12)
            self.assertAlmostEqual(yy, y, places=12)

    def test_derivation_and_independent_validation(self):
        expected = MapAlignment(1.23, -0.42, 0.31)
        scene = [-2.7, -2.5, 3.05]
        mapped = [*expected.scene_to_map(*scene[:2]), scene[2]+expected.yaw]
        actual = MapAlignment.from_poses(scene, mapped)
        second = [4.1, -1.2, -3.1]
        errors = actual.errors(second, [*expected.scene_to_map(*second[:2]), second[2]+expected.yaw])
        self.assertLess(max(errors), 1e-12)
        self.assertGreater(actual.errors(second, [0, 0, 0])[0], 1)

    def test_actual_enu_uses_xy_not_xz(self):
        validate_convention(CONVENTION)
        self.assertEqual(MapAlignment(0, 0, 0).raw_scene_to_map([1, 2, 99]), (1, 2))
        for change in [dict(webots_coordinate_system='NUE'), dict(scene_ground_axes=['x', 'z'])]:
            with self.assertRaises(ValueError):
                validate_convention({**CONVENTION, **change})

    def test_unique_directories_and_serialized_contract(self):
        with tempfile.TemporaryDirectory() as root:
            first, second = unique_output_directory(root), unique_output_directory(root)
            self.assertNotEqual(first, second)
            a = MapAlignment(1, 2, 0.3)
            path = first/'alignment.yaml'
            data = dict(source_frame='scene_graph', target_frame='map', coordinate_convention=CONVENTION,
                        transform=dict(translation=dict(x=a.x, y=a.y), yaw=a.yaw), matrix=a.matrix)
            path.write_text(yaml.safe_dump(data))
            self.assertEqual(MapAlignment.load(path), a)
            data['matrix'][0][2] = 99
            path.write_text(yaml.safe_dump(data))
            with self.assertRaises(ValueError):
                MapAlignment.load(path)

    def test_completion_gate_ignores_false_and_duplicates(self):
        gate = CompletionGate()
        self.assertFalse(gate.accept(False))
        self.assertFalse(gate.accept(False))
        self.assertTrue(gate.accept(True))
        self.assertFalse(gate.accept(True))
        self.assertFalse(gate.accept(False))


if __name__ == '__main__':
    unittest.main()
