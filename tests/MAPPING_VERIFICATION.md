# Jazzy apartment mapping verification

Verified on 2026-09-26 with Webots R2025a and ROS 2 Jazzy.

- The README workspace build completed for simulation_actions, explore_lite_msgs,
  and explore_lite. Existing upstream C++ test: passed (2 reported tests).
- `ROS_DOMAIN_ID=87 python3 tests/test_mapping_explorer.py`: passed against the
  installed workspace packages. Covers planner abort, navigation rejection,
  navigation abort, navigation success, finite blacklisting, and completion.
- Started `tiago_apartment_mapping.launch.py rviz:=false` from a fresh apartment.
  The explorer automatically completed its Nav2 initial scan, selected a frontier,
  checked it through ComputePathToPose, and sent NavigateToPose. No manual
  navigation or spin goals were sent in this acceptance run.
- Supervisor odometry changed from approximately (-2.69, -2.49) to
  (-1.858, -3.179); the SLAM map grew from 55 x 81 to 72 x 85 at 0.05 m/cell.
- `/map`, `/map_metadata`, `/navigate_to_pose`, `/navigate_through_poses`, and
  dynamic `map -> odom` were observed. TF was approximately identity in XY,
  with a -0.096 m Z correction from Webots base height to the 2D map.
- `scripts/save_apartment_map.sh` successfully wrote PGM/YAML in a new unique
  directory. The partial verification map was moved to
  `/tmp/tiago-mapping-verification/`, not retained as a completed apartment map.
- The local inflation radius was raised to 0.30 m during this acceptance run
  after Nav2 warned that its existing 0.25 m setting was smaller than the
  corrected 0.28 m footprint. This override is now part of the launch.

Runtime fixes found by testing: filter the robot's own lidar returns using the
installed laser_filters body-box filter; initialize coverage with a Nav2
half-turn; clear the known robot footprint in the static costmap layer; provide
valid initial wheel joints in the base-only URDF for Jazzy ros2_control.

Limits: this verifies autonomous startup, movement, map growth, and map saving,
not full-apartment coverage. Nav2 DWB progress failures/recoveries were observed;
closed/narrow routes and blacklisted frontiers can leave rooms unmapped. RViz
frontier configuration was added but the GUI was not part of the headless test.
The default is the repository's base-only mapping robot, not the full arm/head
model. No claims are made about real-robot mapping accuracy: SLAM uses simulation
Supervisor odometry and the existing Webots scan-matching workaround.

## Mapping door preparation

The apartment's seven Door PROTOs were inspected, including handed hinge stops
in the referenced Cyberbotics Door.proto. Six interior doors are configured for
opening; exterior `door(1)` on the x=0 wall is excluded. The existing Supervisor
sets direct door hinge positions at runtime and checks angles plus endpoint
velocities for two continuous simulation seconds. World files are unchanged.

`python3 tests/test_mapping_doors.py`: four tests passed, covering opposite hinge signs,
continuous settling after motion, validation before any mutation, timeout for a
closed/obstructed door, and rejection of non-Door nodes.

Webots integration: the readiness waiter received the Supervisor's settled-door
status before SLAM/Nav2/explorer processes started. The explorer subsequently
completed its initial scan. Normal launches pass an empty door configuration to
Webots and do not instantiate the preparation helper, even if the mapping
configuration environment variable was inherited by the launching shell.

The final door-readiness log reported `door: +1.5501 rad` and each of
`door(2)` through `door(6): -1.5501 rad`. A startup race observed during this
change was fixed by waiting for active Nav2 lifecycle states before launching
the explorer. The final run confirmed door readiness, then SLAM startup, then
Nav2 activation before exploration.
The final explorer completed its initial scan, sent frontier goals, and reported
two successful navigation results. The missing-readiness watchdog returned exit
code 1 as expected. Existing Nav2 progress failures remain a coverage limitation.

## Mapping lifecycle stall: unused path smoother

Inspected the runtime `bt_navigator.default_nav_to_pose_bt_xml` value:
`/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`.
This tree computes and follows paths, with recovery behaviors, and contains no
SmoothPath action. The frontier explorer leaves the goal's behavior_tree empty.

Added a mapping-only variant of nav2_bringup 1.3.12 navigation_launch.py with
exactly the path smoother process/component and lifecycle entry removed. All
other nodes, remaps, parameters, and lifecycle order are retained, including
velocity_smoother. Normal navigation still uses the installed launch. Neither
shared Nav2 parameters nor the readiness timeout were changed for this fix.

`python3 tests/test_mapping_navigation_launch.py`: both checks passed. The AST
parity check compares the variant against upstream with only these removals;
the behavior-tree check verifies ComputePathToPose/FollowPath without SmoothPath.

Fresh Webots run (`rviz:=false`) verified:
- planner_server: active [3]
- controller_server: active [3]
- bt_navigator: active [3]
- smoother_server absent from nodes and lifecycle manager node_names
- explore_node connected as a NavigateToPose client
- first automatic goal: (-2.50, -2.55), followed by bt_navigator's
  "Begin navigating" confirmation. Further automatic goals were also sent.

Acceptance log: `/tmp/tiago-no-path-smoother.log`. The verification simulation
was stopped after testing. Full-apartment coverage was not retested for this
narrow lifecycle change.

## Progress monitoring and endpoint quality (2026-09-26)

Replaced the unused `progress_timeout` / hard-coded 120-second total deadline
with displacement plus best-distance milestones. Defaults: 0.15 m progress,
30 simulation seconds since the last milestone, no total limit. Fresh Nav2
remaining-path feedback allows detours; stationary replanning and oscillation
cannot count as progress. Cancellation retains goal ownership until Nav2 returns
a terminal result. Failed endpoints have a 0.5 m spatial exclusion; successful
endpoints are remembered separately. Known-free targets require 0.35 m mapped
obstacle clearance, 0.4 m boundary setback, and 0.6 m minimum travel, followed by
Nav2 path validation. Frontier detection/scoring is still the vendored algorithm.

Navigation inspection found a concrete costmap ordering defect: the local static
layer ran after voxel and inflation layers and overwrote inflated master costs.
The baseline local map had zero inflated cells, while its global map had 5727.
The mapping-only override now orders static, voxel, inflation; runtime confirmed
the list and nonzero inflated costs (roughly 1500–2000 cells in sampled scenes).
No DWB gains, velocity limits, collision-monitor settings, shared Nav2 config,
or normal navigation launch were changed in this fix. This corrects costmap
composition rather than attempting a broad controller retune.

Inspected the base's 0.265 m collision cylinder versus the existing mapping
radius 0.28 m, 0.30 m local inflation, approximately 0.9 m interior door openings,
DWB BaseObstacle scoring, laser filtering, and odometry/command streams.
Collision monitor FootprintApproach is disabled in the existing configuration.
At real stalls, nonzero commands coincided with almost zero Supervisor odometry;
these were not simply explorer timer failures. Some remaining obstructions near
furniture were not resolved to a specific contact geometry. Several different
endpoints can share an obstructed route, so multiple failures can precede a
route change. No claim is made that planner reachability guarantees physical
execution or that every room is reachable with this 2D observation model.

Extended Webots run, without manual navigation goals:
- All six interior doors settled at the configured angles before SLAM started.
- SLAM published `/map`; planner_server, controller_server and bt_navigator were
  checked active [3]; both navigation actions were present.
- 12 NavigateToPose goals sent, four succeeded, seven stalled/canceled, and the
  twelfth was moving when the observation ended.
- One real goal logged progress at elapsed 15.6, 39.0 and 50.6 seconds, then
  stalled and canceled at 81.8 seconds, after 31.2 seconds without progress.
- After repeated obstructed endpoints, a different route produced movement again.
- Observed free area grew from 9.3625 to 52.3225 square metres; known area reached
  57.1925 square metres. Final grid: 244 x 192 at 0.05 m/cell. Grid bounding area
  includes unknown space and is not a coverage percentage.
- About 430 simulation seconds observed. A temporary contact-inspection controller
  interrupted the end of this diagnostic run; it was removed and the owned
  simulation was stopped. Exploration did not report completion. Full-apartment
  coverage is NOT verified.
- Saved partial map: `/tmp/tiago-progress-verification/apartment_partial.yaml`
  and `.pgm`. Logs: `/tmp/tiago-progress-run1.log` and
  `/tmp/tiago-progress-run1-diagnostics.jsonl` (temporary local artifacts).

Validation commands (source ROS and the workspace first):

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select explore_lite --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select explore_lite --ctest-args -R 'test_progress_tracker|test_explore' --output-on-failure
colcon test-result --test-result-base build/explore_lite --verbose
cd src/webots_ros2_simulation
ROS_DOMAIN_ID=88 python3 tests/test_mapping_explorer.py
ROS_DOMAIN_ID=89 python3 tests/test_mapping_progress.py
python3 tests/test_mapping_navigation_launch.py
python3 tests/test_mapping_doors.py
```

Results: eight reported C++ tests passed; both ROS integration tests passed;
three launch checks and four door tests passed. The progress integration drives
the actual explorer with mock Nav2, TF and accelerated ROS time: 140 seconds of
slow progress survives both 30 and 120 seconds, then a freeze triggers cancellation
about 30 seconds after its last milestone and a subsequent goal succeeds.
This complements the real simulator run rather than claiming the mock proves
physical full-apartment coverage.

## Automatic finalization and scene/map alignment (2026-09-26)

The explorer now publishes a reliable transient-local `/exploration_complete`
Bool (false on startup, true once on genuine frontier exhaustion). Its existing
status topic is retained. The extended explorer integration test verifies that
planner rejection, navigation rejection/abort, and pause do not emit completion;
completion is emitted once after the successful/exhausted search sequence, and a
late subscriber receives true.

The mapping launch adds Nav2's map_saver_server and a lifecycle manager for only
that server, plus `scripts/mapping_finalizer.py`. Normal navigation is unchanged.
The finalizer calls `/mapping_map_saver/save_map`, writes into an atomically
created unique directory, derives SE(2) alignment from synchronized poses, and
checks three later samples before writing `alignment.yaml`. Save failure,
unavailable/stale TF, unsupported axes, movement during validation, or excess
alignment drift cannot produce a successful alignment file. Partial output
folders carry `FINALIZATION_FAILED.txt`; the initial waiting phase may fail
before any folder is created.

Coordinate audit:
- CMOC `tools/scene_graph_generator_from_simulation.py`'s get_world_transform
  composes ancestor transforms and stores the raw world position in
  `qualities.location`. It does not swap X/Y/Z axes. This file was only read.
- The mapping world's WorldInfo omits coordinateSystem. Installed Webots R2025a
  `resources/nodes/WorldInfo.wrl` defines its default as ENU: X east, Y north,
  Z up. Runtime Supervisor metadata independently confirms ENU.
- Existing GroundTruthOdom uses position[0:2] for odom X/Y and
  atan2(orientation[3], orientation[0]) for yaw about +Z. The same data generates
  odom->base_link. There is no additional world-to-odom origin shift in mapping.
- SLAM supplies map->odom. The finalizer still derives the complete per-run
  transform: map->base_link times inverse(scene_graph->base_link), at matching
  timestamps. No numeric identity transform is assumed or stored in code.
- Alignment validation measures pose-source consistency, not independent
  localization accuracy. SLAM scan matching/loop closure are disabled in the
  existing simulation configuration, so a near-identity result is expected.

Focused tests (source Jazzy and the built workspace first):

```bash
python3 tests/test_map_alignment.py
ROS_DOMAIN_ID=88 python3 tests/test_mapping_explorer.py
ROS_DOMAIN_ID=89 python3 tests/test_mapping_finalizer.py
python3 tests/test_mapping_navigation_launch.py
python3 tests/test_mapping_doors.py
```

The five pure tests cover known forward/inverse transforms, numerical round
trips, nonidentity pose-based derivation, ENU X/Y versus rejected X/Z, unique
folders, serialization consistency, and duplicate/false completion suppression.
Four ROS finalizer tests cover saving once, later-sample validation, room centers
inside/outside saved-map bounds, failed SaveMap, and rejection of an injected
0.20 m TF shift, and a missing-TF timeout without saving. All pass. Explorer completion/pause/late-subscription integration
and the existing three launch and four door checks pass as well.

Build after changing the C++ explorer:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select explore_lite --cmake-args -DBUILD_TESTING=ON
```


Live apartment acceptance run:
- Started the complete mapping launch with `rviz:=false`; doors settled, SLAM
  and Nav2 activated, map_saver_server reached active, and exploration ran.
- 36 automatic goals were sent, eight succeeded, and 19 stalled. Exploration
  genuinely exhausted its eligible frontiers at about 1273 simulation seconds;
  no completion message was manually injected.
- The finalizer was restarted during testing with the latest implementation and
  the optional room-center input. That input was generated in `/tmp` from this
  exact world using the existing CMOC parser's read-only helpers; CMOC files
  were not modified. The finalizer received the explorer's real completion
  signal and used the real Nav2 SaveMap service.
- Output: `maps/apartment_20260926_132204_jsva0fkg/`, containing `map.yaml`,
  `map.pgm`, and `alignment.yaml`. The pre-existing manual map was untouched.
- Calibration timestamp: 1274.06 simulation seconds. Later validation samples:
  1274.26, 1274.46, 1274.66. Maximum position error **0.0 m**, maximum yaw error
  **0.0 rad**, within the configured 0.05 m / 0.03 rad tolerances.
- This run's computed transform is identity, consistent with the existing
  ground-truth odometry and disabled scan matching. Nonidentity transforms and
  injected drift are tested separately; identity is not hardcoded.
- Eight room centers checked against the saved grid: five inside, three outside
  (BATHROOM_1, ROOM_1, ROOM_2). These bounds checks do not imply free cells or
  complete coverage. Frontier exhaustion included blacklisted routes.
- Logs: `/tmp/tiago-finalization.log` and `/tmp/tiago-finalizer-node.log`.
  The owned simulation and finalizer were stopped after verifying the artifacts.
