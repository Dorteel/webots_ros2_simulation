# TIAGo apartment navigation

## Current Status — 2026-09-19

The Webots apartment simulation launches with the full TIAGo++ model. Both arms tuck into the navigation pose, ros2_control and diff-drive work, and Nav2 launches and plans. Semantic-free navigation works through `/navigate_to_position`; `navigation.launch.py` starts `navigate_to_position_server` automatically.

Startup from the workspace root:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select navigate_to_position
source install/setup.bash
cd src/webots_ros2_simulation
ros2 launch ./launch/navigation.launch.py
```

Send a position goal from another sourced terminal:

```bash
ros2 action send_goal /navigate_to_position navigate_to_position/action/NavigateToPosition \
  "{x: -1.5, y: -1.8, yaw: 1.57, frame_id: map}" --feedback
```

Navigation still needs tuning around local obstacle handling and DWB. Generated `build/`, `install/`, and `log/` directories should live only in `~/ros2_ws`, not inside `src/` or this repository.

## Call robot actions from ROS 2

Build and source the action packages before launching the apartment:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select navigate_to_position simulation_actions
source install/setup.bash
cd src/webots_ros2_simulation
ros2 launch ./launch/navigation.launch.py
```

The existing Webots Supervisor now serves `/pick` and dispatches to `execute_action()`. The TCP CLI still works. In another sourced terminal:

```bash
ros2 action send_goal /pick simulation_actions/action/Pick '{robot: TIAGo, object: "plate(9)"}' --feedback
```

For Python nodes running an `rclpy` executor, import `PickClient` from `simulation_actions.pick_client` and `NavigateToPositionClient` from `navigate_to_position.client_api`; then `await pick_client.pick("TIAGo", "plate(9)")` or `await navigation_client.navigate(x, y, yaw)`. Both return a boolean. Future place/open/close adapters can use the same Supervisor action pattern; navigation continues through `/navigate_to_position`.

## Map the full apartment

Mapping uses the base-only TIAGo at the same Webots spawn and lidar position as navigation. It runs SLAM Toolbox with the installed TIAGo settings and does not start Nav2. From the repository root:

```bash
ros2 launch ./launch/make_map.launch.py
```

Drive from another sourced terminal with `python3 arrow_teleop.py`. Explore the whole apartment until the gray unknown areas in RViz are covered, then save to a new name:

```bash
ros2 run nav2_map_server map_saver_cli -f "$(pwd)/maps/apartment"
```

This creates `maps/apartment.yaml` and `maps/apartment.pgm` without replacing the kitchen map. Mapping uses the same Webots axes and world origin as navigation; SLAM owns `map -> odom` during mapping. If the saved map is used for semantic navigation, verify its map-frame alignment and update `config/semantic_map_generation.yaml`'s Webots-to-map transform before regenerating semantic poses.

## Generate semantic navigation map

From the repository root, run `python3 scripts/generate_semantic_navigation_map.py`. It extracts 228 Webots objects, converts their poses to `map`, and samples map-checked approach poses for the anchor types in `config/semantic_map_generation.yaml`. The transform there matches the current ground-truth TF anchor: TIAGo starts at Webots `(-2.69, -2.49, 0)` and map `(-2.63, -2.26, 0)`, giving translation `(0.06, 0.23)` and zero yaw offset. Update that configuration if the spawn or map origin changes. Static occupancy checks cannot guarantee Nav2 can reach a pose.

Inspect `config/semantic_navigation_map.yaml` and the poses in RViz. Use the marker below to correct problematic objects; manual poses are preserved on the next generation run. A future object-name resolver can pass the preferred pose to `/navigate_to_position`.

## Mark semantic navigation objects

Use two terminals so the marker can read object names interactively:

```bash
# Terminal 1: from this repository root
ros2 launch ./launch/semantic_mapping.launch.py
```

```bash
# Terminal 2: from this repository root, with ROS 2 sourced
python3 scripts/semantic_map_marker.py
```

Enter an object name (for example, `fridge`), select **Publish Point** in RViz, then click the robot approach position and the object position. Repeat for more objects; each pair is saved immediately to `config/semantic_navigation_map.yaml`. Enter `q`, `quit`, or `exit` to stop. Existing names require overwrite confirmation.

## Next Steps

1. **Semantic navigation map:** define symbolic objects (fridge, table, sink, etc.) and one or more approach poses per object. Store at least `x`, `y`, `yaw`, and `frame_id`, so `move_to_object("fridge")` can later resolve object → pose → `/navigate_to_position`.
2. **Remaining ROS2 Actions:** expose pick, place, place_in_container, open, close, and other supported fallback actions. Reuse their existing implementation logic.
3. **Action execution node:** expose a clean interface for higher-level symbolic actions, translate symbolic parameters into ROS2 Action calls, and keep planning independent of Webots/controller details. Eventually execute generated plans sequentially.

```kotlin
symbolic plan
     ↓
action execution / knowledge interface
     ↓
ROS2 Actions
     ├── navigate_to_position
     ├── pick
     ├── place
     ├── open
     └── ...
     ↓
Nav2 / Webots controllers
```
