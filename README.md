# Generic and TIAGo++ Webots simulations

Two launchers share the Webots/ROS environment while keeping their controllers
and interfaces separate:

| Launcher | Default and capabilities |
|---|---|
| `./run_simulation.sh` | Multi-robot apartment with human, TIAGo and randomized TurtleBot, using independent hardware/sensor interfaces; `--profile generic` selects the empty world. No legacy actions or navigation. |
| `./run_tiago_simulation.sh` | Original TIAGo apartment, RGB/depth/lidar, wheel control, Supervisor pick/place/open/close and Look/LookAhead/Turn actions; optional navigation and mapping. |

`run_simulation.sh` defaults to `multi-robot` and also accepts `--profile generic`.
`run_tiago_simulation.sh` retains `basic`, `navigation`, `mapping`, `--navigation`,
`--check-only`, `--build-only`, and existing `name:=value` launch arguments.
Old legacy commands must now use the TIAGo launcher; the generic launcher rejects
legacy profile flags with a migration message. Neither launcher changes action
semantics. Use separate `ROS_DOMAIN_ID` values if running both workflows at once,
since each world publishes `/clock` and legacy interfaces use global names.

See [REPOSITORY_GUIDE.md](REPOSITORY_GUIDE.md) for the legacy architecture,
[dependency profiles](dependencies/README.md) for package responsibilities,
and the generic sections below for per-robot interfaces.

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
./install-dependencies.sh --profile generic  # generic and multi-robot workflows
./install-dependencies.sh            # original TIAGo basic workflow
# Optional, cumulative profiles:
./install-dependencies.sh --navigation
./install-dependencies.sh --exploration  # navigation + SLAM + vendored explorer
```

`install-dependencies.sh` detects ROS_DISTRO, requires Jazzy, updates rosdep and
installs dependencies from the selected `package.xml` files, including tests.
It leaves package-manager confirmation enabled and is safe to repeat.
`--profile basic|navigation|mapping|generic|multi-robot` selects a profile; `--check` checks
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

Generic simulation:

```bash
./run_simulation.sh                      # multi-robot apartment (default)
./run_simulation.sh --profile generic    # empty world
./run_simulation.sh --profile multi-robot # apartment with two generic drivers
./run_simulation.sh --check-only
./run_simulation.sh --build-only
```

Original TIAGo simulation:

```bash
./run_tiago_simulation.sh --check-only     # prerequisites; no build
./run_tiago_simulation.sh --build-only     # build legacy packages; no launch
./run_tiago_simulation.sh                  # full robot, RGB/depth, manual wheel control
./run_tiago_simulation.sh --navigation     # add Nav2, position action and RViz
./run_tiago_simulation.sh --profile mapping  # base-only SLAM/exploration; no cameras
```

Both launchers validate prerequisites **before building**, incrementally build
only the selected profile, source the overlay and reuse existing launch files.
Ctrl-C shuts down only its own launch process group. Run one legacy apartment at a time;
its Supervisor reserves TCP port 8765. Build/install/log directories default to
this checkout and are ignored by Git. To use another workspace, consistently
set `SIM_WORKSPACE=/absolute/workspace` for run and test commands. `ROS_SETUP`
can select a nonstandard Jazzy setup file.

Pass existing launch arguments as `name:=value`, for example:

```bash
./run_tiago_simulation.sh world:="$PWD/worlds/complete_apartment_tiago_ros2.wbt"
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
./run_tiago_simulation.sh
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
`./run_tiago_simulation.sh look_turn_config:=/absolute/path/to/look_turn.yaml`.
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

## Generic per-robot simulation (Phase 1)

The `generic` profile starts a floor, lighting and a small **clock-only** controller.
It has no predefined physical robots, centralized discovery, navigation or
apartment dependencies. Each added robot runs an independent
`webots_ros2_driver` instance with the same `GenericRobot` Python plugin. The
plugin enumerates its own devices through `getNumberOfDevices()` and
`getDeviceByIndex()`, reads position sensors and commands ordinary Motor APIs.
It neither uses Supervisor scene operations nor depends on SMPA.

```bash
./install-dependencies.sh --profile generic  # if dependencies are missing
./run_simulation.sh --profile generic --check-only
./run_simulation.sh --profile generic
```

To add robots, pause Webots and add Robot nodes or robot PROTOs under the root
`children`. Give each a **unique name**, set its `controller` to `"<extern>"`, and
save the world under this repository's `worlds/` directory. Keep the `world clock`
node. Close Webots and relaunch, supplying the names (no joint/device configuration
is required):

```bash
./run_simulation.sh --profile generic world:="$PWD/worlds/my_robots.wbt" \
  'robots:=["robot_a","robot_b"]'
```

Robot namespaces are `<lowercase-type>_<number>`, with independent counters
starting at 1 per normalized type. Plain names are shorthand for both the Webots
name and type: `TIAGo` becomes `/tiago_1`, and `TurtleBot` becomes `/turtlebot_1`.
For different Webots names belonging to the same type, supply the type explicitly
in the existing `robots` list (it is not guessed from the model or name):

```bash
./run_simulation.sh --profile generic world:="$PWD/worlds/my_robots.wbt" \
  'robots:=[{"name":"alpha","type":"Tiago"},{"name":"beta","type":"Tiago"}]'
```

This assigns `/tiago_1` to `alpha` and `/tiago_2` to `beta`. Names are sorted before
numbering, so reordering the launch list does not change assignments. Type labels
are lowercased, unsafe characters become underscores, and labels beginning with
a digit receive a `robot_` prefix. Types that normalize identically share a
counter. Webots names must still be unique for external-controller attachment.
Changing robot names/types or the launched robot set can change numbering.
Restart/relaunch after hardware changes; hot-plugging is not implemented. The
launcher's namespace map is passed to each generic plugin through its process
environment, keeping hardware and native sensor publishers in the same namespace.
These drivers replace the robots' original controllers; models requiring special
initialization may need additional plugins.

Inspect interfaces in another terminal, using the domain printed by the launcher:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0                  # match the launcher
ros2 node list --no-daemon
ros2 topic list --no-daemon -t
# Exact namespace for a robot named robot_a:
ROBOT_NS=/robot_a_1
ros2 topic echo "$ROBOT_NS/hardware_info" std_msgs/msg/String \
  --once --qos-durability transient_local --qos-depth 1
ros2 topic echo "$ROBOT_NS/joint_states" sensor_msgs/msg/JointState --once
```

Interfaces under each namespace:

- `hardware_info`: retained `std_msgs/String` with versioned JSON describing all
  enumerated devices, motor limits, associated position sensors, exact command
  topics, descriptive `motor_id` values and the `joint_feedback` name-to-sensor mapping. ROS has no standard
  hardware-inventory message; this avoids custom message infrastructure.
- `joint_states`: `sensor_msgs/JointState` with measured finite positions, stamped
  in simulation time. Sensorless motors do not get invented feedback. Passive
  position sensors publish feedback without a command topic. Velocity and effort
  arrays remain empty. Joints with neither motor nor sensor cannot be discovered
  through the device API.
- `motors/<motor_id>/position`: `std_msgs/Float64`, immediate position target
  in radians or meters. Out-of-range and nonfinite values are rejected. Position
  speed is capped at the smaller of the motor maximum and `positionSpeed` (default
  0.5 rad/s or m/s in `config/generic_robot.urdf`).
- `motors/<motor_id>/velocity`: `std_msgs/Float64`, offered only for motors
  without position bounds. Uses ordinary velocity mode and rejects values above
  the motor's maximum speed. Commands must be refreshed within `velocityTimeout`
  (default 0.5 simulation seconds), otherwise velocity is set to zero.
- `command_status`: `std_msgs/String` JSON reporting acceptance, rejection or a
  velocity timeout. Acceptance means the API command was applied, not that the
  target was reached; use `joint_states` for feedback.
- `driver/<device-name>/...`: native driver interfaces. Camera defaults include
  `image_color` and `camera_info`; depth, lidar, distance sensors, GPS, compass,
  and other supported devices retain their upstream message types and behavior.
  Most sensor streams activate when subscribed. Use `ros2 topic list -t` to find
  their actual names. The shared `/clock` is published once per world.

Motor topic names come from the original Webots device names, converted to
lowercase with invalid ROS characters replaced by underscores. Names starting
with a digit get a `motor_` prefix. Collisions receive a numeric suffix (`_2`,
`_3`, etc.) in device discovery order, only when needed to keep IDs unique within
each robot. For example, `head_1_joint` stays `head_1_joint`, and `right wheel motor`
becomes `right_wheel_motor`.
Original names remain in `hardware_info.devices[].name` and `joint_states.name`.
Use `motor_id`, `position_sensor`, `joint_feedback` and `command_topics` to map
an exposed topic to its physical joint and feedback sensor.

For TIAGo's discovered `head_1_joint`, **check its limits and units in
`hardware_info`, and its current position in `joint_states`** before commanding:

```bash
ROBOT_NS=/tiago_1
ros2 topic echo "$ROBOT_NS/command_status" std_msgs/msg/String
# In another sourced terminal with the same ROS_DOMAIN_ID:
ROBOT_NS=/tiago_1
ros2 topic pub --once \
  "$ROBOT_NS/motors/head_1_joint/position" \
  std_msgs/msg/Float64 '{data: 0.15}'
```

No motor setters run during discovery. Native sensor plugins, image encoding,
calibration, transport, external-controller attachment and simulation stepping
are reused from `webots_ros2_driver`. The small plugin fills its generic
motor/position-sensor gap; it does not duplicate camera or lidar publishers.
The `.urdf` file is only the driver's plugin configuration, with no fabricated
links or joints, and is not published as a physical `robot_description`.

Limitations: bounded motors support position commands only; coupled motors and
invalid limits are inventoried but not commanded. Torque/force commands,
trajectory scheduling, `ros2_control`, TF generation and automatic IMU assembly
are not implemented. Other unsupported device types remain inventory-only.
Native driver sensor naming rules still apply: use unique device names with
letters, digits and underscores, beginning with a letter; names that collide
after upstream sanitization are unsupported. Native frame IDs are not prefixed
by this plugin. Run one generic world per ROS domain. An explicitly accepted
position target stays active until replaced; only velocity commands have a
watchdog. The next control step is a validated `ros2_control` hardware adapter
and standard controllers, using the same real device ownership and feedback.

## Generic TIAGo Joint Control

The minimal demonstration reuses the stock TIAGo++ PROTO with `supervisor FALSE`
and the unchanged generic driver. It starts no apartment controllers or action
servers. Launch from the repository (the PROTO/assets need network access on first
use unless already cached):

```bash
ROS_DOMAIN_ID=90 ./run_simulation.sh --profile generic \
  world:="$PWD/worlds/generic_tiago.wbt" 'robots:=["TIAGo"]'
```

In another terminal, discover the namespace and inspect devices, limits, sensor
associations and measured positions:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=90
ros2 node list --no-daemon
ros2 topic list --no-daemon -t
ROBOT_NS=/tiago_1
ros2 topic echo "$ROBOT_NS/hardware_info" std_msgs/msg/String \
  --once --qos-durability transient_local --qos-depth 1
ros2 topic echo "$ROBOT_NS/joint_states" sensor_msgs/msg/JointState --once
```

Live discovery found 56 devices and 23 commandable motors, including:

| Motor | Position sensor | Position limits (rad) | Command mode |
|---|---|---|---|
| `head_1_joint` | `head_1_joint_sensor` | −1.24 to 1.24 | position |
| `head_2_joint` | `head_2_joint_sensor` | −0.98 to 0.79 | position |

The head motor topic names preserve `head_1_joint` and `head_2_joint`.
Confirm their command topics in `hardware_info` after any hardware changes.
Both motors report a 3 rad/s maximum; the generic position-command speed cap is
0.5 rad/s. Their discovered position topics are:

```text
/tiago_1/motors/head_1_joint/position
/tiago_1/motors/head_2_joint/position
```

Record the initial head positions first. For this fresh world, where both are
approximately zero, command the small absolute pan target below:

```bash
HEAD_TOPIC="$ROBOT_NS/motors/head_1_joint/position"
ros2 topic pub --once "$HEAD_TOPIC" std_msgs/msg/Float64 '{data: 0.1}'
ros2 topic echo "$ROBOT_NS/joint_states" sensor_msgs/msg/JointState --once
# Return near the fresh-world initial position; substitute your recorded value
# if you started from a different position.
ros2 topic pub --once "$HEAD_TOPIC" std_msgs/msg/Float64 '{data: 0.0}'
```

To monitor acceptance/rejection, run `ros2 topic echo
/tiago_1/command_status std_msgs/msg/String` in a separate sourced
terminal **before** publishing commands; status is not retained. Joint-state
positions, rather than command acceptance, confirm motion. Stop the simulation
with **Ctrl-C in the launch terminal**.

Live verification on 2026-10-27, using only the generic position topic:

| Measurement | `head_1_joint` (rad) |
|---|---:|
| Initial | −0.000000650848 |
| Commanded (initial + 0.10) | 0.099999349152 |
| Measured after movement | 0.099999346353 |
| Measured after restoring initial target | −0.000000652173 |

The return error was below 1.4×10⁻⁹ rad. A 1.34-rad target was explicitly rejected
as outside the discovered limits, with no subsequent head movement. The other
head joint's maximum deviation during the move was below 9×10⁻⁹ rad. The sole
command subscriber was this robot's `hardware` node; no other actuator was
commanded. Small physical changes occurred in uncommanded joints (largest arm
change about 0.00015 rad), so this does not imply an entirely motionless chassis
or whole-robot pose validation. No generic-controller changes were needed.

## Multi-Robot Apartment Simulation

```bash
./run_simulation.sh                         # same multi-robot default
./run_simulation.sh --profile multi-robot
./run_simulation.sh --profile multi-robot seed:=11  # reproducible initial pose
```

This loads `worlds/generic_multi_robot_apartment.wbt`, copied from the
setting-the-table apartment with its objects and human preserved. TIAGo and a
stock TurtleBot3 Burger named `TurtleBot` each use the unchanged generic driver.
The old action Supervisor and external ROS Supervisor are replaced by the
clock-only controller. No navigation, manipulation or SMPA packages are started.
Stop with **Ctrl-C in the launch terminal**.

TurtleBot is placed **before Webots starts**, by writing a temporary world beside
the source (preserving relative asset/controller paths). It samples center
coordinates uniformly from x ∈ [−9.0, −8.5], y ∈ [−5.2, −4.4] m, z = 0, and yaw
from [−π, π]. This living-room region was checked against the preserved furniture,
walls, human and TIAGo locations, allowing a 0.15 m footprint radius plus at least
0.35 m clearance. The world declares these center bounds in its
`GENERIC_SPAWN_REGION` comment. No live robot teleportation or Supervisor is used.
The chosen seed and pose are printed; omit `seed` to generate a fresh seed.
Temporary world/project files are removed on normal launch exit.

Optional `world:="$PWD/worlds/custom_apartment.wbt"` uses another world with the
same `DEF TURTLEBOT TurtleBot3Burger` header, translation/rotation fields and an
explicitly validated `GENERIC_SPAWN_REGION` comment. Revalidate the region after
changing obstacles or the robot footprint: this is not automatic mesh collision
detection, and lidar checks only cover its scan plane. Seeds reproduce initial
poses, not every subsequent physics or sensor sample. A forced kill can leave
temporary `.generic_spawn_*.wbt` files and their hidden `.wbproj` companions.

Inspect both robots in another terminal (match the launcher's domain; default 0):

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
TIAGO_NS=/tiago_1
TURTLE_NS=/turtlebot_1
ros2 node list --no-daemon
ros2 topic list --no-daemon -t
for ROBOT_NS in "$TIAGO_NS" "$TURTLE_NS"; do
  ros2 topic echo "$ROBOT_NS/hardware_info" std_msgs/msg/String \
    --once --qos-durability transient_local --qos-depth 1
  ros2 topic echo "$ROBOT_NS/joint_states" sensor_msgs/msg/JointState --once
done
# Native sensor samples (TurtleBot has lidar, not a camera):
ros2 topic echo "$TURTLE_NS/driver/LDS_01" sensor_msgs/msg/LaserScan --once
ros2 topic echo "$TIAGO_NS/driver/Astra_rgb/image_color" sensor_msgs/msg/Image \
  --once --field encoding
```

`hardware_info` maps each descriptive `motor_id` to its original name, command
topics, limits, units and feedback sensor. In this world, TIAGo head pan is
`head_1_joint`; TurtleBot right wheel is `right_wheel_motor` (original device name
`right wheel motor`). In the fresh world, after checking the current positions
and limits, these small **absolute** position targets exercise independent motors:

```bash
HEAD_TOPIC="$TIAGO_NS/motors/head_1_joint/position"
WHEEL_TOPIC="$TURTLE_NS/motors/right_wheel_motor/position"
ros2 topic pub --once "$HEAD_TOPIC" std_msgs/msg/Float64 '{data: 0.1}'
ros2 topic pub --once "$WHEEL_TOPIC" std_msgs/msg/Float64 '{data: 0.12}'
ros2 topic echo "$TIAGO_NS/joint_states" sensor_msgs/msg/JointState --once
ros2 topic echo "$TURTLE_NS/joint_states" sensor_msgs/msg/JointState --once
# Restore approximately zero; substitute the initial values you recorded:
ros2 topic pub --once "$HEAD_TOPIC" std_msgs/msg/Float64 '{data: 0.0}'
ros2 topic pub --once "$WHEEL_TOPIC" std_msgs/msg/Float64 '{data: 0.0}'
```

Monitor `<namespace>/command_status` before publishing to see acceptance or
rejection. The wheel command causes a small base movement; it is not a navigation
command. Wheel position is unbounded, but wheel velocity commands are limited to
±6.67 rad/s and use the generic watchdog. Do not assume all listed motors have
feedback: the lidar's two internal motors have no position sensors.

Live verification on 2026-10-27:

| Seed/run | Initial x, y (m) | Yaw (rad) | Observed minimum TurtleBot lidar range |
|---|---|---|---|
| 11 | −8.773810223245, −4.752182091136 | 2.665393708688 | 1.1190 m |
| 22 | −8.520895310091, −5.087705127939 | −2.993208024875 | 0.9461 m |
| 11 repeated | −8.773810223245, −4.752182091136 | 2.665393708688 | 1.1186 m |

Initial poses above are the launch-generated world fields; there is no new
Supervisor pose/odometry interface. All three live launches ran with both drivers,
advancing `/clock`, wheel feedback and nonempty lidar scans. The first run also
received TIAGo's 667-beam lidar and 640×480 BGRA camera messages. TIAGo reported
57 devices/23 commandable motors; TurtleBot reported 10 devices/4 motors and two
wheel feedback sensors. Accelerometer and gyro remain inventory-only under the
existing generic/native driver configuration.

TIAGo's head moved from −0.000000409079 to 0.099999578376 rad for a
0.099999590921-rad target. TurtleBot's right wheel moved from 0 to 0.11932 rad for
a 0.12-rad target (sensor resolution 0.00628 rad), then returned to 0. TIAGo's head
also returned to its initial position within 2×10⁻⁹ rad. The other robot's monitored
head/wheel coordinates did not change during either command. Each command topic
had only its own hardware node subscribed; checked clock/joint/lidar topics had
one publisher each. An out-of-bounds head target and a 7.67-rad/s wheel command
were rejected without changing their targets.

Environment limitations observed: the original human has an empty trajectory
argument, so its pedestrian controller exits and the human remains stationary.
This is preserved, not replaced with a new behavior. The installed Webots lacks
some local appearance/CapScrew PROTOs referenced by existing connector assets;
the first run also reported remote visual-asset download failures. Physics and
ROS checks succeeded, but complete visual parity is not certified on this
installation. A complete Webots asset installation/cache and working network
access remain prerequisites for full rendering. Existing apartment mass-ratio
warnings also remain. No generic-controller changes or test infrastructure were
introduced for this integration.
