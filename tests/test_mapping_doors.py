"""Door preparation checks with Webots node/field doubles (no simulator needed)."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       'controllers/fallback_action_supervisor'))
from mapping_doors import MappingDoors, door_hinge


def field(value, method):
    result = MagicMock()
    getattr(result, method).return_value = value
    return result


def door(left):
    position = field(0.0, 'getSFFloat')
    params = MagicMock()
    params.getField.side_effect = {
        'position': position,
        'minStop': field(-1.57 if left else -0.01, 'getSFFloat'),
        'maxStop': field(0.01 if left else 1.57, 'getSFFloat'),
    }.get
    leaf = MagicMock()
    leaf.getVelocity.return_value = [0.0] * 6
    hinge = MagicMock()
    hinge.getTypeName.return_value = 'HingeJoint'
    hinge.getField.side_effect = {
        'jointParameters': field(params, 'getSFNode'),
        'endPoint': field(leaf, 'getSFNode'),
    }.get
    hinge.setJointPosition.side_effect = lambda target, axis: setattr(
        position.getSFFloat, 'return_value', target)
    result = MagicMock()
    result.getTypeName.return_value = 'Door'
    result.getField.side_effect = {
        'canBeOpen': field(True, 'getSFBool'),
        'selfClosing': field(False, 'getSFBool'),
        'jointAtLeft': field(left, 'getSFBool'),
    }.get
    children = MagicMock()
    children.getCount.return_value = 1
    children.getMFNode.return_value = hinge
    result.getBaseNodeField.return_value = children
    return result, hinge, position, leaf


class DoorPreparationTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.config = Path(self.temp.name) / 'doors.json'
        self.config.write_text(json.dumps(dict(
            doors=['left', 'right'], open_angle=1.55, angle_tolerance=0.08,
            settle_seconds=2.0, timeout_seconds=20.0)))
        self.supervisor = MagicMock()
        self.supervisor.getTime.return_value = 0.0
        self.node = MagicMock()
        self.left, self.right = door(True), door(False)

    def create(self, nodes=None):
        nodes = nodes if nodes is not None else dict(left=self.left[0], right=self.right[0])
        with patch('mapping_doors.get_node', side_effect=lambda _, name, kind: nodes[name]):
            return MappingDoors(self.supervisor, self.node, self.config, '/test/doors')

    def status(self):
        return self.node.create_publisher.return_value.publish.call_args.args[0].data

    def test_signed_opening_requires_continuous_settling(self):
        preparation = self.create()
        self.left[1].setJointPosition.assert_called_once_with(-1.55, 1)
        self.right[1].setJointPosition.assert_called_once_with(1.55, 1)
        preparation.step()
        self.supervisor.getTime.return_value = 1.0
        self.left[3].getVelocity.return_value = [0, 0, 0, 0, 0, 0.1]
        preparation.step()
        self.left[3].getVelocity.return_value = [0] * 6
        self.supervisor.getTime.return_value = 2.0
        preparation.step()
        self.supervisor.getTime.return_value = 3.9
        preparation.step()
        self.assertFalse(preparation.done)
        self.supervisor.getTime.return_value = 4.0
        preparation.step()
        self.assertTrue(self.status().startswith('ready: '))
        self.assertEqual(json.loads(self.status()[7:]), {'left': -1.55, 'right': 1.55})

    def test_missing_door_prevents_all_mutations(self):
        preparation = self.create({'left': self.left[0]})
        self.assertTrue(preparation.done)
        self.assertTrue(self.status().startswith('error:'))
        self.left[1].setJointPosition.assert_not_called()

    def test_closed_or_obstructed_door_times_out(self):
        preparation = self.create()
        self.left[2].getSFFloat.return_value = 0.0
        self.supervisor.getTime.return_value = 21.0
        preparation.step()
        self.assertTrue(self.status().startswith('error:'))
        self.assertIn('left', self.status())

    def test_non_door_is_rejected(self):
        self.left[0].getTypeName.return_value = 'Cabinet'
        with self.assertRaises(ValueError):
            door_hinge(self.left[0])


if __name__ == '__main__':
    unittest.main()
