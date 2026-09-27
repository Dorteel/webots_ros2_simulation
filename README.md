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

## VLM observation belongs to CMOC

Simulation publishes RGB, depth, and CameraInfo; CMOC interprets sensor data.
The observation server and interface have moved to `cmoc/ros2/cmoc_perception`
and `cmoc/ros2/cmoc_interfaces`. This repository no longer owns or launches a
VLM server; its other simulated robot actions remain in `simulation_actions`.
See the [CMOC observation instructions](../cmoc/README.md#observe-one-camera-frame-with-a-vlm)
for building, backend parameters, and example action goals.

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

## Automatically map the apartment

The automatic launch starts the existing base-only TIAGo mapping apartment, SLAM Toolbox, the existing
Nav2 stack with `nav2_params_jazzy.yaml`, and the vendored Jazzy-built
`explore_lite` frontier explorer. No static map server, AMCL, or fixed `map → odom`
anchor is started. SLAM owns `map → odom`; the existing Webots Supervisor owns
`/odom` and `odom → base_link`. The lidar frame remains `hokuyo`. Mapping alone remaps the native scan to
`/scan_raw` and uses the standard `laser_filters/LaserScanBoxFilter` to remove
returns inside TIAGo’s body box before publishing `/scan`. This avoids mapping
the robot itself as obstacles. The filter box is configured in
`config/apartment_scan_filter.yaml`; it does not clear space behind those returns.

Dependencies (already present on the development machine except the vendored
explorer, which is built below):

```bash
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-webots-ros2 ros-jazzy-laser-filters
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select simulation_actions explore_lite_msgs explore_lite
source install/setup.bash
cd src/webots_ros2_simulation
ros2 launch ./launch/tiago_apartment_mapping.launch.py
```

Mapping uses `launch/mapping_navigation.launch.py`, a Jazzy Nav2 bringup variant
that omits only the unused path `smoother_server` process/component and lifecycle
entry. The default `navigate_to_pose_w_replanning_and_recovery.xml` tree runs
`ComputePathToPose` and `FollowPath` without `SmoothPath`, so this avoids blocking
mapping startup on an unnecessary smoother lifecycle transition. The command
`velocity_smoother` remains enabled. Normal `navigation.launch.py` still includes
the installed Nav2 launch with its path smoother. No lifecycle timeout was raised.

Check this narrow launch difference and behavior-tree dependency:

```bash
source /opt/ros/jazzy/setup.bash
python3 tests/test_mapping_navigation_launch.py
```

With automatic mapping running, verify activation and goal dispatch:

```bash
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
# Each should print active [3].
ros2 param get /lifecycle_manager_navigation node_names
# smoother_server is absent; velocity_smoother remains.
ros2 action info /navigate_to_pose
ros2 topic echo /navigate_to_pose/_action/status --once
```

The launch log should show `Nav2 active; starting frontier exploration`, followed
by `Navigation goal sent` after the initial scan.

Before SLAM starts, the existing Webots Supervisor opens the six interior Door
PROTOs in `config/apartment_mapping_doors.json`: `door`, `door(2)`, `door(3)`,
`door(4)`, `door(5)`, and `door(6)`. The exterior entrance `door(1)` on the x=0
boundary remains closed. Each door's direct hinge is set to ±1.55 radians,
respecting its handedness and joint limits. Readback must remain within 0.08 rad
of the target, with linear speed below 0.02 m/s and angular speed below 0.05 rad/s,
for two continuous simulation seconds. Only then are SLAM, Nav2, and the explorer
started. The explorer additionally waits for the Nav2 controller, planner,
behavior server, and navigator to become active before its initial scan. The
startup log reports measured door angles.

Missing/unsupported doors or failure to settle within 20 simulation seconds
abort automatic mapping; a 180-second wall-clock watchdog also handles a missing
Supervisor or stalled simulation. No map is started with partly prepared doors.
For another world, supply `doors_config:=/absolute/path/to/doors.json` with its
interior door names and settling settings. Door preparation applies even with
`explore:=false` in this launch. It changes runtime joint positions only: no
world files are saved, no generic open/close actions are changed, and normal
simulation/manual mapping launches retain their original door states.

Use `rviz:=false` to omit RViz, `explore:=false` for manual mapping, or
`world:=/absolute/path/to/apartment.wbt` for another compatible apartment world (pair with `robot_urdf:=...` when needed).
Run one apartment/navigation launch at a time. This follows the repository's
existing file-based launch convention. No manual navigation goals are needed.
The explorer waits for the SLAM map and TF, performs a collision-checked
Nav2 half-turn to fill the initial lidar blind area, then searches the known/unknown
boundary, validates each target using Nav2 `ComputePathToPose`, and sends the
reachable path endpoint to `NavigateToPose`. Thus it reuses obstacle inflation,
planner and controller settings. Mapping overrides the costmap radius to
the base’s 0.28 m radius (with local inflation raised from 0.25 m to 0.30 m)
and enables static-layer footprint clearing to connect
the occupied robot footprint to observed free space. Unknown-space planning
remains disabled. The local costmap runs static, voxel, then inflation layers:
putting static last erased the inflation costs used by DWB.

Endpoints are known free cells at least 0.6 m from the robot, set back 0.4 m
from the unknown boundary with 0.35 m clearance from mapped obstacles. Nav2
validates each endpoint before navigation. Failed endpoints are excluded within
0.5 m for the session; successful endpoints are tracked separately to avoid
repeated visits. Other endpoints on the same frontier can still be explored.

`config/apartment_explore.yaml` controls progress: `progress_timeout: 30.0`
means 30 simulation seconds without a meaningful progress milestone. A milestone
requires at least `progress_distance: 0.15` m of robot displacement plus a 0.15 m
improvement in best distance to the goal or fresh Nav2 remaining-path distance.
Stationary replanning and oscillation do not reset the timer. Moving goals can
run longer than 30 or 120 seconds; `navigation_timeout: 0.0` disables a total
limit (set a positive value to enable one). A stall cancels navigation, blacklists
the endpoint, waits for the terminal action result, then tries another candidate.
Progress logs are limited to one per 10 simulation seconds. Completion means no
useful unvisited, unblacklisted endpoints remain, not guaranteed full coverage.
The initial scan has a 45-second action allowance;
if it fails, exploration pauses instead of declaring completion. RViz displays `/explore/frontiers`.

Verify from another sourced terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /map --once --field info
ros2 topic echo /map_metadata --once
ros2 run tf2_ros tf2_echo map odom
# Ctrl-C the TF monitor before continuing.
ros2 action list | grep navigate
ros2 topic echo /navigate_to_pose/_action/status --once
ros2 topic echo /cmd_vel --once
ros2 topic echo /odom --once --field pose.pose.position
ros2 topic echo /explore/status --once --qos-durability transient_local
```

Watch the launch terminal for exploration started, frontier selected, navigation
goal sent, goal succeeded/failed, and exploration complete. Status values are in
`third_party/explore_lite_msgs/msg/ExploreStatus.msg`. Compare odometry samples
to verify movement. Pause/resume exploration without shutting down SLAM:

```bash
ros2 topic pub --once /explore/resume std_msgs/msg/Bool '{data: false}'
ros2 topic pub --once /explore/resume std_msgs/msg/Bool '{data: true}'
```

On genuine exploration completion, the launch automatically saves:

```text
maps/apartment_<UTC timestamp>_<unique suffix>/
├── map.yaml
├── map.pgm
└── alignment.yaml
```

The explorer publishes `/exploration_complete` (`std_msgs/Bool`, reliable,
transient-local): false at startup and true once after exhausting useful frontier
endpoints. Rejected/aborted goals, blacklisting, and pause do not publish true.
Completion still means frontier exhaustion, not guaranteed full-apartment coverage.
The finalizer ignores repeated completion messages. With `explore:=false`, the
automatic saver/finalizer are not started.

`mapping_finalizer.py` waits for a stationary robot, calls the lifecycle-managed
Nav2 `/mapping_map_saver/save_map` service, then validates and saves the alignment.
Directories are created atomically with unique names; existing maps are untouched.
The log reports `Mapping finalized: <directory>` on success. A save/TF/convention/
validation error logs `Mapping finalization FAILED`; a created directory receives
`FINALIZATION_FAILED.txt` and no valid `alignment.yaml`. A 90-second wall-time
watchdog also handles a paused simulation. Leave the launch running until the
finalization result appears. A partial directory is not a finalized map.

**Coordinate convention:** the inspected scene-graph generator composes world
transforms and stores raw Webots `[X,Y,Z]` in `qualities.location`. This apartment
uses Webots ENU (X east, Y north, Z up), so the ground plane is **X/Y**, not X/Z.
The existing Supervisor publishes these X/Y values directly in `/odom` and
`odom → base_link`, with yaw `atan2(R[3], R[0])` about +Z. It now also reports its
actual WorldInfo convention on `/mapping/coordinate_convention` in mapping mode.
The finalizer verifies that metadata and the odometry publisher identity. Other
axis conventions are rejected until an explicit matching odometry conversion
is implemented; axes are never guessed from the filename.

The per-run alignment is
`T_map_scene = T_map_base × inverse(T_scene_base)`.
It uses TF at the exact timestamp of the Supervisor odometry sample; no alignment
from a previous run is reused. Three later samples must agree within **0.05 m**
and **0.03 rad**, and odometry must match `odom → base_link` at the same timestamp.
`alignment.yaml` includes the transform, 3×3 matrix, world/convention metadata,
calibration poses/timestamp, and measured errors from all validation samples.
This tests consistency of the pose sources, not absolute SLAM accuracy.

The ROS-independent utility accepts planar scene X/Y coordinates:

```python
from scripts.map_alignment import MapAlignment
alignment = MapAlignment.load("maps/apartment_<run>/alignment.yaml")
x_map, y_map = alignment.scene_to_map(x_scene, y_scene)
x_scene, y_scene = alignment.map_to_scene(x_map, y_map)
# For a raw scene-graph qualities.location = [X,Y,Z]:
x_map, y_map = alignment.raw_scene_to_map([x_scene, y_scene, z_scene])
```

Optional launch arguments: `maps_directory:=/absolute/output/directory` and
`scene_graph:=/absolute/path/to/current.scene_graph.json`. If a scene graph is
supplied, room locations (`type: Location`) are checked against the **saved**
occupancy-grid bounds and recorded in the alignment metadata. Outside-grid room
centers are reported, not rejected; no assumption is made that centers are free.
The scene graph must describe the selected world's unmodified raw coordinates.

The manual `./scripts/save_apartment_map.sh` remains available for intermediate
map-only snapshots; it does not produce alignment metadata. To choose a path
manually, ensure it is unused before running
`ros2 run nav2_map_server map_saver_cli -f /absolute/unused/path/apartment`.

The default reuses the existing base-only mapping robot. Near-body lidar
self-returns blocked unfiltered frontier paths in testing. No arms or cameras are present in this mapping
variant. The full-body navigation launch is unchanged.

Coverage is limited to reachable free space and useful frontiers at least 0.5 m
long. Closed doors, narrow gaps, failed/blacklisted targets, and the existing
Nav2 radius/controller settings can leave unknown regions. “Complete” means no
remaining eligible frontier, not proof that every room was observed. As in the
installed Webots TIAGo SLAM configuration, scan matching is disabled for the
known Webots lidar issue; this simulation relies on Supervisor ground-truth
odometry, and loop closure is disabled to avoid inconsistent corrections.
Map coordinates must not be assumed to match the static kitchen/semantic map.

Regression test (in a separate, unused DDS domain after building):

```bash
ROS_DOMAIN_ID=87 python3 tests/test_mapping_explorer.py
ROS_DOMAIN_ID=89 python3 tests/test_mapping_progress.py
```

These check planner failures, navigation rejection/abort/success, blacklisting,
and completion using deterministic ROS action servers and an occupancy grid,
plus a goal that progresses for 140 simulated seconds before stalling and yielding
to another goal. See [verification results](tests/MAPPING_VERIFICATION.md) for
the extended Webots run, build/test commands, and measured coverage limits.

The old `ros2 launch ./launch/make_map.launch.py` remains available for base-only
manual mapping with `python3 arrow_teleop.py`.

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
