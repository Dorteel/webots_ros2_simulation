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

## TIAGo RGB and depth cameras

The default apartment driver (`config/tiago_webots_wheels.urdf`, also used by
navigation) publishes the existing head-mounted `Astra rgb` Camera and
`Astra depth` RangeFinder through the native `webots_ros2_driver` sensor plugins. No additional node or build is needed.
Start it from this repository in a ROS 2 Jazzy sourced terminal:

```bash
ros2 launch ./launch/tiago_apartment_ros2.launch.py
```

In another sourced terminal:

```bash
ros2 topic list | grep -Ei 'camera|depth'
ros2 topic hz /tiago/camera/color/image_raw
ros2 topic echo /tiago/camera/color/image_raw --once
# Stop each hz command with Ctrl-C before running the next command.
ros2 topic hz /tiago/camera/depth/image_raw
ros2 topic echo /tiago/camera/depth/image_raw --once
ros2 topic echo /tiago/camera/depth/camera_info --once
ros2 topic echo /tiago/camera/color/camera_info --once
# Optional visual inspection:
ros2 run rqt_image_view rqt_image_view
```

`/tiago/camera/color/image_raw` is `sensor_msgs/msg/Image`, with native `bgra8`
bytes (blue, green, red, alpha), Webots-reported dimensions (normally 640×480),
simulation timestamps, and frame ID `Astra_rgb`. The camera samples every
simulation timestep; wall-clock publishing frequency depends on simulation speed.
Camera calibration is available at `/tiago/camera/color/camera_info`.
`/tiago/camera/depth/image_raw` is `sensor_msgs/msg/Image` with `32FC1` depth
in meters, Webots-reported dimensions (normally 640×480), simulation timestamps,
and frame ID `Astra_depth`. It also samples every simulation timestep, without
unit conversion. `/tiago/camera/depth/camera_info` provides `sensor_msgs/msg/CameraInfo`
with the depth intrinsics (`K` and `P` matrices). Out-of-range depth values can be
infinite; consumers should check for finite, valid depth values.

These are raw RGB and depth streams: the Astra sensors have a 26 mm offset and
are not registered to each other, so an RGB pixel must not be assumed to address
the same scene point in the depth image. No registration or 3D projection is
implemented here. The native RangeFinder plugin additionally exposes its built-in
`/tiago/camera/depth/point_cloud` topic.
The base-only mapping robot has no head camera.

## Observe one camera frame with a VLM

`simulation_actions` provides `/observe_with_vlm` with action type
`simulation_actions/action/ObserveWithVLM`. It continuously caches the latest RGB
image and sends exactly one snapshot plus the goal prompt to the selected backend.
The result contains `success` and plain-text `response`; feedback reports
`running_vlm`. Missing images, unavailable endpoints, and invalid responses abort
the goal with `success: false` and an explanation. Frames continue updating during
inference. Concurrent goals are rejected while one inference is active; canceling
an in-flight HTTP request is not supported. The cached frame can be old if the camera stops publishing.

Build and launch the camera simulation (terminal 1):

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select simulation_actions
source install/setup.bash
cd src/webots_ros2_simulation
ros2 launch ./launch/tiago_apartment_ros2.launch.py
```

Start Ollama (`ollama serve` if it is not already running), install a vision model,
and start the action server (terminal 2):

```bash
ollama pull qwen3-vl:2b
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run simulation_actions observe_with_vlm_server --ros-args -p backend:=ollama
```

Test from terminal 3:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 action list
ros2 action send_goal /observe_with_vlm simulation_actions/action/ObserveWithVLM \
  "{prompt: 'Describe what you see.'}" --feedback
ros2 action send_goal /observe_with_vlm simulation_actions/action/ObserveWithVLM \
  "{prompt: 'Do you see a coffee mug?'}" --feedback
```

To use Nebula, stop the action server in terminal 2, set the key in that shell,
and restart it (the key is never a ROS parameter):

```bash
read -rsp 'Nebula API key: ' NEBULA_API_KEY; echo
export NEBULA_API_KEY
ros2 run simulation_actions observe_with_vlm_server --ros-args -p backend:=nebula
```

Use the same goal commands for either backend. Nebula receives the snapshot and
prompt over HTTPS. If the server was started with the required environment key,
you can also switch between goals with
`ros2 param set /observe_with_vlm_server backend nebula` (or `ollama`).

| Parameter | Default |
| --- | --- |
| `backend` | `ollama` |
| `camera_topic` | `/tiago/camera/color/image_raw` |
| `ollama_url` | `http://localhost:11434` |
| `ollama_model` | `qwen3-vl:2b` |
| `nebula_url` | `https://nebula.cs.vu.nl/api/chat/completions` |
| `nebula_model` | `SURF.Qwen3.5 122B A10B NVFP4` |
| `nebula_api_key_env` | `NEBULA_API_KEY` |
| `request_timeout_sec` | `120.0` |

Pass overrides using `--ros-args -p name:=value`. Set `camera_topic` at startup;
changing it requires restarting the node. HTTP calls do not stream or retry.
The HTTP timeout bounds network waits; allow more time for slow local inference.

To reproduce failure checks, stop the server and run one of these alternatives,
then send a goal with the commands above. The last three need a camera frame:

```bash
# No camera frame:
ros2 run simulation_actions observe_with_vlm_server --ros-args -p camera_topic:=/unused_camera
# Ollama unavailable:
ros2 run simulation_actions observe_with_vlm_server --ros-args -p ollama_url:=http://127.0.0.1:1
# Nebula missing key:
env -u NEBULA_API_KEY ros2 run simulation_actions observe_with_vlm_server --ros-args -p backend:=nebula
# Nebula unavailable (with the key exported):
ros2 run simulation_actions observe_with_vlm_server --ros-args -p backend:=nebula -p nebula_url:=http://127.0.0.1:1
```

Run the ROS/HTTP integration tests after sourcing the built workspace:

```bash
cd ~/ros2_ws/src/webots_ros2_simulation
ROS_DOMAIN_ID=81 python3 -m unittest discover -s simulation_actions/test -v
```

## Save and replay a Nav2 path

With the navigation launch running, compute a route from the saved semantic poses for `cabinet(1)` and `plate1`:

```bash
python3 scripts/save_nav_path.py cabinet1_to_plate1 \
  --semantic-map config/setting_the_table_map.yaml \
  --from-object 'cabinet(1)' --to-object plate1
```

The result is `config/paths/cabinet1_to_plate1.yaml`. Replay it from near the saved start pose:

```bash
ros2 run navigate_to_position follow_saved_path config/paths/cabinet1_to_plate1.yaml
```

For Python code, `await FollowSavedPathClient(node).follow(path_file)` returns a boolean; import the class from `navigate_to_position.follow_saved_path_client`. Numeric poses are also accepted with `--start X Y YAW --goal X Y YAW`. Saved paths do not replan around changed obstacles.

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
