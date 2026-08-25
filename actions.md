# Actions
- move(robot, coordinates): teleports the robot to coordinates x, y, z
- move_to_object(robot, object, distance=0.8, clearance=0.15): teleports the robot to the nearest clear sampled pose at or beyond the requested distance
- pick(robot, object): teleports the objects into the robot's end effector
- place(robot, object, coordinates): teleports the object onto the location shown by the coordinates
- open(robot, object): Assuming the object is a HingeJoint, the joint is set to 80 degrees
- close(robot, object): Assuming the object is a HingeJoint, the joint is set to 0 degrees
