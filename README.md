# TIAGo++ apartment simulation

Webots simulates a TIAGo++ robot with RGB/depth cameras, lidar and wheel control.
Optional ROS 2 Nav2 navigation and SLAM/frontier exploration build on the same
apartment. A Webots Supervisor provides ground-truth odometry and simulated
pick/place/open/close actions. See [REPOSITORY_GUIDE.md](REPOSITORY_GUIDE.md) for
the architecture and interfaces, [dependency profiles](dependencies/README.md)
for package responsibilities, and [VALIDATION.md](VALIDATION.md) for measured results.

## Requirements

- Ubuntu **24.04 amd64**, ROS 2 **Jazzy**, Python 3.12 and a C/C++ build toolchain.
- Webots **R2025a** (revisions accepted) and a compatible Jazzy Webots ROS driver.
- Working display/OpenGL access for Webots and camera rendering.
- Network access for APT, rosdep and initially uncached Webots PROTO assets.

Basic simulation does **not** require Nav2, SLAM or exploration packages.
Navigation and mapping are explicit dependency/build profiles. These scripts
select a release, not a frozen snapshot of every Ubuntu/ROS package version.

## Fresh installation

First install ROS 2 Jazzy following the [official Ubuntu instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
Use a clean Jazzy shell. From this checkout, bootstrap the installer/build tools:

```bash
sudo apt update
sudo apt install build-essential cmake python3-colcon-common-extensions python3-rosdep
source /opt/ros/jazzy/setup.bash
sudo rosdep init                     # once per machine; omit if already initialized
```

Then run the repository installers **manually**:

```bash
./install-webots.sh                  # interactive confirmation before system changes
./install-dependencies.sh            # basic robot, sensors and control
# Optional, cumulative profiles:
./install-dependencies.sh --navigation
./install-dependencies.sh --exploration  # navigation + SLAM + vendored explorer
```

`install-dependencies.sh` detects ROS_DISTRO, requires Jazzy, updates rosdep and
installs dependencies from the selected `package.xml` files, including tests.
It leaves package-manager confirmation enabled and is safe to repeat.
`--profile basic|navigation|mapping` selects the same profiles; `--check` checks
without installation or network updates. No launcher installs dependencies.

`install-webots.sh` follows the [official Cyberbotics APT method](https://cyberbotics.com/doc/guide/installation-procedure):
HTTPS signing key, a dedicated `signed-by` source and `apt` installation. Type
`INSTALL` at its prompt to authorize APT configuration/package changes. It
refuses conflicting existing Cyberbotics sources, never edits unrelated sources,
and verifies the installed Webots and `ros-jazzy-webots-ros2` bridge. It installs
an available R2025a package explicitly; if that release is no longer available,
it stops instead of silently choosing another version. A compatible existing
installation is retained. No `apt-key` is used.

## Manual Webots installation

As an alternative to `install-webots.sh`, follow Cyberbotics' APT instructions
above with a key in `/etc/apt/keyrings` and a `signed-by` source; inspect
`apt-cache madison webots` and select R2025a explicitly. If APT no longer offers
it, download the amd64 R2025a `.deb` from the
[official release](https://github.com/cyberbotics/webots/releases/tag/R2025a):

```bash
sudo apt install ./webots_2025a_amd64.deb  # use the exact downloaded filename
sudo apt install ros-jazzy-webots-ros2
```

For an extracted R2025a tarball or another installation location, point the
standard driver resolver at its home directory:

```bash
export ROS2_WEBOTS_HOME=/absolute/path/to/webots
# WEBOTS_HOME=/absolute/path/to/webots is also supported.
python3 scripts/webots_path.py
```

`ROS2_WEBOTS_HOME` takes precedence over `WEBOTS_HOME`; otherwise the ROS driver
searches its standard installation locations, including the official APT layout.
Existing `/usr/local/webots` installations remain supported. Set overrides in
your shell startup file only when needed. No launcher path is hardcoded.

## Run

```bash
./run_simulation.sh --check-only     # packages, Webots, executables and config; no build
./run_simulation.sh                  # full robot, RGB/depth, manual wheel control
./run_simulation.sh --navigation     # add Nav2, position action and RViz
./run_simulation.sh --profile mapping  # base-only SLAM/exploration; no cameras
```

The launcher validates prerequisites **before building**, incrementally builds
only the selected profile, sources the overlay and reuses existing launch files.
Ctrl-C shuts down only its own launch process group. Run one apartment at a time;
the Supervisor reserves TCP port 8765. Build/install/log directories default to
this checkout and are ignored by Git. To use another workspace, consistently
set `SIM_WORKSPACE=/absolute/workspace` for run and test commands. `ROS_SETUP`
can select a nonstandard Jazzy setup file.

Pass existing launch arguments as `name:=value`, for example:

```bash
./run_simulation.sh world:="$PWD/worlds/complete_apartment_tiago_ros2.wbt"
```

## Cameras and control

In another terminal from the checkout, source the same workspace and use the
DDS domain printed by the launcher:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash             # or $SIM_WORKSPACE/install/setup.bash
export ROS_DOMAIN_ID=42               # replace with the launcher's domain
ros2 topic hz /tiago/camera/color/image_raw
ros2 topic echo /tiago/camera/depth/image_raw --once --field encoding --qos-reliability best_effort
python3 arrow_teleop.py               # arrows drive; q exits and sends stop
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped '{}'
```

RGB uses `bgra8`; depth uses `32FC1` meters. Both provide `camera_info` under their
color/depth namespaces. They are not registered to each other. Optional GUI:
`sudo apt install ros-jazzy-rqt-image-view`, then
`ros2 run rqt_image_view rqt_image_view`. No GUI viewer is required by the robot.
Use manual control without competing Nav2 commands. Navigation/action examples
and limitations are in the guide; [advanced workflows](docs/ADVANCED_USAGE.md)
cover semantic maps and saved paths.

## Tests

```bash
./test.sh                            # basic prerequisites, build, unit tests and live smoke test
./test.sh --all                      # also build/test optional navigation and exploration
SIM_TEST_DOMAIN_ID=88 ./test.sh       # choose another unused DDS domain
```

Tests require Webots/display access. They refuse an occupied DDS domain, check
camera buffers and simulation time, command a small in-place rotation, measure
odometry, send stop and clean up their own processes. The full suite also runs
the existing mapping regressions. Unavailable prerequisites return **2** before
building; build failures/timeouts return **3**, regression failures/timeouts
**4**, and live sensor/control/shutdown failures **5**. Other setup errors are
nonzero. Logs are in `log/` (or `$SIM_WORKSPACE/log/`), including
`simulation-test.log`. A successful dependency check is not a live test pass.

## Troubleshooting

- **Missing package / module:** run `./install-dependencies.sh --check` with the
  same profile, then install that profile. Source Jazzy in a clean shell;
  do not mix ROS distributions. A successful interface build alone does not
  establish that runtime dependencies exist.
- **rosdep has no database / resolution fails:** bootstrap `python3-rosdep`, run
  `sudo rosdep init` once and `rosdep update --rosdistro jazzy`. Check network,
  proxy and ROS APT configuration. Installers stop clearly on network failures.
- **Webots missing or wrong version:** run `python3 scripts/webots_path.py`;
  inspect/unset stale `ROS2_WEBOTS_HOME` and `WEBOTS_HOME`. R2025a is required;
  a newer driver's minimum version may require a matching older Jazzy driver.
- **Duplicate Cyberbotics source / signed-by conflict:** inspect existing files
  in `/etc/apt/sources.list.d` and consolidate them manually. The installer
  deliberately refuses to overwrite another configuration.
- **No camera frames / GUI fails:** check DISPLAY/Wayland and OpenGL access,
  Webots logs and whether simulation time is advancing. The mapping robot has
  no cameras. Headless execution needs a configured display/rendering backend;
  disabling rendering prevents this camera test from working.
- **DDS socket errors / no graph:** use matching domains and permit local DDS
  networking. Restricted sandboxes may deny `getifaddrs` or UDP sockets.
- **Port 8765 occupied:** stop the other apartment launch; scripts do not kill
  unrelated processes. Missing configuration files and map images are reported
  by preflight before the build.

## Look and turn actions

The common Supervisor serves `/look_left`, `/look_right` (`Look`),
`/look_ahead` (`LookAhead`), and `/turn_left`, `/turn_right` (`Turn`) from
`simulation_actions/action/`. Use the full robot in basic simulation:

```bash
./run_simulation.sh
# Another terminal, same ROS_DOMAIN_ID:
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 action list -t
ros2 action send_goal /look_left simulation_actions/action/Look '{}' --feedback
ros2 action send_goal /look_right simulation_actions/action/Look '{}' --feedback
ros2 action send_goal /look_ahead simulation_actions/action/LookAhead '{}' --feedback
ros2 action send_goal /turn_left simulation_actions/action/Turn '{}' --feedback
ros2 action send_goal /turn_right simulation_actions/action/Turn '{}' --feedback
ros2 action send_goal /look_left simulation_actions/action/Look '{angle: 0.8}' --feedback
ros2 action send_goal /turn_right simulation_actions/action/Turn '{angle: 1.2}' --feedback
```

Angles are nonnegative radians. **Zero selects the configured default**, rather
than zero movement; negative/nonfinite goals are rejected. Left is positive pan
or yaw, right is negative. Look actions move only the head, clamping relative pan
to mechanical limits. Look-ahead resets both pan and tilt. Results report the
absolute achieved displacement; feedback reports absolute remaining angle.
The head uses the existing Supervisor joint positioning mechanism, with readback
on subsequent simulation steps, rather than an arm/head trajectory controller.
Cancellation cannot undo an already applied head movement.

`config/look_turn.yaml` sets look default (30 degrees), neutral pan/tilt,
look tolerance/timeout, turn default (90 degrees), maximum angular velocity
(0.4 rad/s), turn tolerance (0.03 rad), timeout, odometry freshness timeout and
stall timeout. Timeouts use wall time and are evaluated each Supervisor step.
An entirely paused/stuck Webots process cannot process goals or cancellation
until stepping resumes. Restart the simulation after configuration edits, or use
`./run_simulation.sh look_turn_config:=/absolute/path/to/look_turn.yaml`.
`TIAGO_LOOK_TURN_CONFIG` also selects the file when running the Supervisor directly.

Turns command the real simulated wheel controller through its discovered
`Twist`/`TwistStamped` subscription and integrate `/odom` yaw across wraparound.
Stamped commands use simulation odometry timestamps. They slow down near the
requested angle and explicitly send zero on completion, cancellation, failure,
and orderly shutdown. Loss of the controller/odom, lack of progress, or a
competing velocity publisher aborts the turn. Stop teleop before turning.

Coordination is deliberately conservative: **turns reject while Nav2 or explorer
nodes are present, even if idle**. Stop the navigation/mapping launch and use the
basic launch for turns. If navigation appears during a turn, the action aborts
and sends zero. This is graph-based exclusion, not an atomic command arbiter;
do not launch another velocity controller during a turn. All five actions share
one reservation; conflicting goals and legacy mutating TCP/interaction requests
are rejected while it is held. Existing TCP `gaze` semantics are unchanged.
Base-only mapping robots have no head joints and return a head-action failure.

Minimal awaitable Python helper (keep the caller's ROS executor spinning):

```python
from simulation_actions.look_turn_client import LookTurnClient
robot = LookTurnClient(node)
await robot.look_left()             # configured default
await robot.look_right(angle=0.8)
await robot.look_ahead()
await robot.turn_left()
await robot.turn_right(angle=1.2)
robot.destroy()
```

Run `./test.sh` or `./test.sh --all`. Both include the look/turn regressions and,
after the basic sensor smoke check, `scripts/check_look_turn.py` for live action,
direction, isolation, cancellation/stop and legacy-discovery checks. Deterministic
ROS tests simulate head hardware and odometry; they do not prove Webots physics.
