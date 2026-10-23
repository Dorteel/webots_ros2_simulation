# Dependency audit and profiles

The original four ROS packages own their library/interface dependencies. Root
launch files, Webots controllers and utility scripts were not part of any ROS
package, so their runtime dependencies were invisible to rosdep. Three minimal
ament packages now describe those root-level components. They install only
package metadata; existing launches remain file-based. No dependency is copied
into the installation script: `scripts/environment.sh` selects source paths,
and rosdep reads their manifests.

| Profile | Source packages selected | Intended use |
|---|---|---|
| basic (default) | `simulation_actions`, `tiago_simulation_bringup` | Full robot, RGB/depth/lidar, wheel control, Supervisor actions |
| navigation | basic + `navigate_to_position`, `tiago_navigation_bringup` | Nav2, RViz, semantic-map tools and saved paths |
| mapping | navigation + `explore_lite_msgs`, `explore_lite`, `tiago_mapping_bringup` | SLAM, exploration, map saving and full regressions |

## Existing packages

- **simulation_actions**: `ament_cmake`, `ament_cmake_python` and
  `rosidl_default_generators` at build time; `rosidl_default_runtime`, `rclpy`,
  `action_msgs` at execution. Generated action fields are primitives. It needs
  neither Nav2 nor Webots to build/use its interfaces.
- **navigate_to_position**: same build tools/generator/runtime; execution also
  uses `geometry_msgs`, `nav_msgs`, `nav2_msgs`, `action_msgs`, `rclpy` and
  `python3-yaml`. Its runtime genuinely requires Nav2 interfaces, even though
  primitive action generation can build without them. Declaring only a build
  dependency would not address its runtime requirement.
- **explore_lite_msgs**: `ament_cmake` and `rosidl_default_generators` at build
  time; `rosidl_default_runtime` at execution. Its status message uses strings.
- **explore_lite**: build/export/runtime dependencies on `explore_lite_msgs`,
  `geometry_msgs`, `map_msgs`, `nav2_costmap_2d`, `nav2_msgs`, `nav_msgs`,
  `rclcpp`, `rclcpp_action`, `std_msgs`, `tf2`, `tf2_ros`, `visualization_msgs`.
  Its launch file also requires `ament_index_python`, `launch`, `launch_ros`.
  Tests require `ament_cmake_gtest`, `ament_lint_auto`, `ament_lint_common`;
  the missing gtest declaration is added. Unused direct `sensor_msgs` and
  `tf2_geometry_msgs` declarations/CMake lookups are removed after checking all
  source/header/test includes.

`ament_cmake_python` is now a build-tool dependency in both client/interface
packages. No optional navigation dependencies were added to `simulation_actions`.
No additional CMake test dependencies were invented for packages with no tests.

## Root component ownership

`tiago_simulation_bringup` declares Webots driver/control, wheel controllers,
controller manager, robot_state_publisher, TF, launch infrastructure, ROS Python,
sensor/geometry/navigation/std/clock message types, YAML, the local action package
and CLI extensions used by launch/control commands. `nav_msgs` is required here
for odometry; it does not imply the Nav2 navigation stack. Standard-library
Python modules and Webots' own `controller` Python module are not pip dependencies.
The latter ships with Webots itself, installed separately by the official APT
method. Optional `rqt_image_view` is not required by the simulation.

`tiago_navigation_bringup` declares the basic profile, position wrapper,
`nav2_bringup`, `nav2_common`, map server/lifecycle manager, `laser_filters`,
RViz. Semantic-map utilities require `python3-numpy`
and `python3-pil`. Nav2 bringup owns its internal planners/controllers/behaviors;
their dependencies are resolved transitively, not copied into our manifests.

`tiago_mapping_bringup` declares navigation, SLAM Toolbox, the local explorer,
`webots_ros2_tiago` (the legacy manual mapping launch reads its SLAM resource),
`lifecycle_msgs` and the lifecycle CLI. `tf2_msgs` is a test dependency for
mapping-finalizer regression subscriptions. Other test imports are already
runtime dependencies of these packages/profiles. Tests use Python's standard
`unittest`; no pytest dependency is introduced.

All three metadata packages build with `ament_cmake`. Profiles are cumulative;
the mapping profile includes full navigation utilities for consistency with the
existing mapping/navigation regressions. This is a functional profile split,
not a minimal package subset for every individual mapping tool.

## Installation boundaries

`install-dependencies.sh` uses rosdep and the configured ROS/Ubuntu package
repositories. Bootstrap ROS, colcon, rosdep and the compiler as documented in
the main README. Repeat execution does not reinstall already satisfied packages.
Read-only `--check` uses the local rosdep database and does not update it.

`install-webots.sh` owns only these dedicated APT configuration paths:
`/etc/apt/keyrings/tiago-cyberbotics.asc` and
`/etc/apt/sources.list.d/tiago-cyberbotics.list`. Existing differing files or
other Cyberbotics sources require manual review. The script confirms before
system changes, downloads the official key over HTTPS, checks its key format,
and lets APT verify repository signatures. It accepts R2025a revisions, retains
compatible installs, and does not automatically downgrade newer installations.
It does not pin/hold future system-wide upgrades or freeze all ROS package
versions. For byte-for-byte reproducibility, a versioned system/container image
and a populated Webots asset cache remain future work.
