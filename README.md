# TIAGo apartment navigation

From the repository root, source ROS 2 Jazzy and start the apartment, Nav2, map server, and RViz:

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch ./launch/navigation.launch.py
```

The launch uses `maps/kitchen.yaml` and simulation time. The Supervisor anchors ground-truth odometry to the initial map pose stored in `nav2_params_jazzy.yaml`; AMCL is not launched.
