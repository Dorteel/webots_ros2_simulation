# TIAGo apartment navigation

From the repository root, source ROS 2 Jazzy and start the apartment, Nav2, map server, and RViz:

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch ./launch/navigation.launch.py
```

The launch uses `maps/kitchen.yaml` and simulation time. The Supervisor anchors ground-truth odometry to the initial map pose stored in `nav2_params_jazzy.yaml`; AMCL is not launched.

## Simple navigation action

Build the separate action package once from this repository root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select navigate_to_position
source install/setup.bash
```

Keep `ros2 launch ./launch/navigation.launch.py` running. In another sourced terminal, start the wrapper:

```bash
ros2 run navigate_to_position navigate_to_position_server --ros-args -p use_sim_time:=true
```

Send a goal (an empty `frame_id` also means `map`):

```bash
ros2 action send_goal /navigate_to_position navigate_to_position/action/NavigateToPosition \
  "{x: -1.5, y: -1.8, yaw: 1.57, frame_id: map}" --feedback
```

Another Python program can use the small example client:

```bash
ros2 run navigate_to_position navigate_to_position_client -1.5 -1.8 1.57
```

The wrapper forwards each goal to Nav2's `/navigate_to_pose` action; its result reports whether Nav2 reached the goal.
