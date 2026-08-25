"""
Contents
--------
set_pose() - Commands TIAGo's stationary tucked-arm pose.
enable_cameras() - Enables TIAGo's RGB and depth cameras.
main()     - Starts and maintains the idle controller.
"""

from controller import Robot


IDLE_POSE = {
    "head_1_joint": 0.0,
    "head_2_joint": 0.0,
    "torso_lift_joint": 0.15,
    "arm_right_1_joint": -1.10,
    "arm_right_2_joint": 1.4679,
    "arm_right_3_joint": 2.714,
    "arm_right_4_joint": 1.7095,
    "arm_right_5_joint": -1.5708,
    "arm_right_6_joint": 1.3898,
    "arm_right_7_joint": 0.0,
    "arm_left_1_joint": -1.10,
    "arm_left_2_joint": 1.4679,
    "arm_left_3_joint": 2.714,
    "arm_left_4_joint": 1.7095,
    "arm_left_5_joint": -1.5708,
    "arm_left_6_joint": 1.3898,
    "arm_left_7_joint": 0.0,
}


def set_pose(robot):
    """Command all upper-body motors to the idle pose."""
    for name, position in IDLE_POSE.items():
        motor = robot.getDevice(name)
        motor.setVelocity(motor.getMaxVelocity() / 2.0)
        motor.setPosition(position)

    for name in ("wheel_left_joint", "wheel_right_joint"):
        wheel = robot.getDevice(name)
        wheel.setPosition(float("inf"))
        wheel.setVelocity(0.0)


def enable_cameras(robot, timestep):
    """Enable TIAGo's RGB camera and depth range finder."""
    robot.getDevice("Astra rgb").enable(timestep)
    robot.getDevice("Astra depth").enable(timestep)


def main():
    """Initialize TIAGo and hold the commanded pose without animation."""
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    set_pose(robot)
    enable_cameras(robot, timestep)
    while robot.step(timestep) != -1:
        pass


if __name__ == "__main__":
    main()
