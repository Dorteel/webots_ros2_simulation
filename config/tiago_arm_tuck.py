"""Tuck both TIAGo++ arms using the existing idle pose."""

from pathlib import Path
import sys

# Reuse the pose already used by the standalone Webots TIAGo controller.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'controllers' / 'tiago_idle'))
from tiago_idle import IDLE_POSE  # noqa: E402


class TiagoArmTuck:
    def init(self, webots_node, properties):
        robot = webots_node.robot
        for name, position in IDLE_POSE.items():
            if not name.startswith('arm_'):
                continue
            motor = robot.getDevice(name)
            motor.setVelocity(motor.getMaxVelocity() / 2.0)
            motor.setPosition(position)

    def step(self):
        pass
