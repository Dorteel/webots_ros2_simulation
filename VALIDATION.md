# Validation — environment reproducibility update, 2026-10-15

The environment has changed since the earlier audit below. **No installation
was performed by the agent.** Neither the Webots installer nor rosdep install
was executed. The dependency install script was exercised only with `--check`.
Current installed packages were discovered, not installed by this task:

```text
Webots resources/version.txt: R2025a
webots: 2025a
ros-jazzy-webots-ros2: 2025.0.0-1noble.20261006.043136
ros-jazzy-nav2-msgs: 1.3.13-1noble.20260921.023049
Resolved Webots: /usr/local/webots/webots
```

### Current results

| Check | Measured result |
|---|---|
| ROS/package/Webots preflight | Basic and mapping profiles pass rosdep, package discovery, Python imports, executable and configuration checks |
| Build | **All seven packages build**, including all original packages and three dependency-profile packages |
| Python regressions | Full discovered suite at time of run: **21 tests pass**. After adding the real-driver precedence case, the environment suite separately reports **4 tests pass** |
| Explorer C++ tests | Both gtest executables pass: **1 + 5 test cases** |
| Full colcon test results | **Fail**: aggregate reports 58 tests, 37 failures, 10 skipped (includes CTest wrapper results, not 58 independent functional cases). Failures are flake8, pep257, uncrustify and xmllint checks in the vendored explorer |
| Manifest validation | All seven manifests pass `catkin_pkg` parsing/validation |
| Unavailable environment/configuration | Wrong ROS distribution and nonexistent world preflights both exit **2**, before building |
| Shell/Python static checks | Shell syntax, Python compilation and whitespace diff checks pass |
| Basic live test | **Fails with exit 5** after building two basic packages and passing 12 selected unit tests in that run; required live messages do not arrive before the timeout |
| Webots startup | Executable starts; world remains loading, with asset connection errors. Controllers report that the simulation world is not ready |
| Cameras/control/time | **Not verified**. Probe times out before issuing any motion command |
| Cleanup | Owned launch processes are stopped and absent in a process-ID check. Webots logs **segmentation fault / exit 139 during shutdown**, so this is not a clean simulator shutdown |
| Installation scripts | Reviewed and syntax-checked only; system-change/download/install paths remain **unexecuted** |

The xmllint failure occurs while retrieving the ROS package schema:
`failed to load external entity "http://download.ros.org/schema/package_format3.xsd"`.
It persists when colcon tests are rerun outside the sandbox. Manifest semantic
validation succeeds locally; no lint checks were disabled to hide these failures.
The source files implicated by style linters were not reformatted as part of
this dependency task.

The first restricted live attempt encountered `getifaddrs: Operation not
permitted` and DDS UDP socket restrictions. The subsequent run used approved
access outside the sandbox and reached Webots. World loading still failed;
network access to uncached Webots assets remains a runtime prerequisite even
when all apt/rosdep dependencies are satisfied.

### Exact commands for the current checks

From the repository root:

```bash
./run_simulation.sh --check-only
./run_simulation.sh --profile mapping --check-only
./install-dependencies.sh --profile mapping --check
SIM_WORKSPACE=/tmp/tiago-repro-full ./run_simulation.sh --profile mapping --build-only
SIM_WORKSPACE=/tmp/tiago-repro-unrestricted ./test.sh
SIM_WORKSPACE=/tmp/tiago-repro-full ./test.sh --all
```

The live test above ran outside the restricted sandbox; the build itself did
not need elevated privileges. Logs: `/tmp/tiago-repro-unrestricted-test.log` and
`/tmp/tiago-repro-unrestricted/log/simulation-test.log`. The default commands
write under the selected workspace's `log/` directory.

```bash
source /opt/ros/jazzy/setup.bash
source /tmp/tiago-repro-full/install/setup.bash
colcon --log-base /tmp/tiago-repro-full/log test \
  --base-paths simulation_actions navigate_to_position third_party/explore \
  third_party/explore_lite_msgs dependencies \
  --build-base /tmp/tiago-repro-full/build \
  --install-base /tmp/tiago-repro-full/install
colcon test-result --test-result-base /tmp/tiago-repro-full/build --verbose
ROS_DOMAIN_ID=88 timeout --signal=INT --kill-after=10s 300s \
  python3 -m unittest discover -s tests -p 'test_*.py' -v
python3 -m unittest discover -s tests -p test_environment.py -v
ROS_DISTRO=humble ./run_simulation.sh --check-only
bash -n install-dependencies.sh install-webots.sh run_simulation.sh test.sh scripts/environment.sh
python3 -m py_compile scripts/preflight.py scripts/webots_path.py scripts/check_simulation.py launch/tiago_apartment_ros2.launch.py
git diff --check
```

Colcon test-result exits nonzero on the lint results; Python regression commands
exit 0. Wrong-distribution preflight exits 2 as expected. The new environment
unit tests cover profile separation, wrong-distribution rejection, missing/
unsupported/non-executable Webots, and actual driver precedence for
`ROS2_WEBOTS_HOME` versus `WEBOTS_HOME`. No fake Webots executable is launched.

`SIM_WORKSPACE=/tmp/tiago-repro-full ./test.sh --all` was executed outside the
sandbox and returned **4** at the explorer lint failures, before starting another
simulation. Its log is `/tmp/tiago-repro-all-test.log`. The full Python suite was
therefore also run independently, as shown above.
Fixing the unrelated explorer formatting, making schema validation independent
of external HTTP, and supplying an offline asset cache remain follow-up work.
The successful install paths and live camera/control checks still need manual
validation on a fresh, graphics-enabled machine with functioning asset access.

---

## Earlier audit snapshot — 2026-10-15

Repository inspected before edits. Host has `/opt/ros/jazzy`, colcon and a display
variable `DISPLAY=:1`; this does not establish functioning graphics. No existing
repository changes were present. No dependencies were installed or existing
simulation/ROS processes stopped. Generated validation files live in
`/tmp/tiago-validation` (temporary, not portable evidence).

## Results

| Check | Result and evidence |
|---|---|
| All four packages build | **Fails**: `explore_lite` CMake cannot find `nav2_msgs` |
| Interface/client packages build | **Verified**: `navigate_to_position`, `simulation_actions`, `explore_lite_msgs` build in a separate selected-package run |
| Colcon tests on those three packages | Command succeeds but reports **0 tests**; this is not functional test coverage |
| Existing Python regressions | **9 tests pass, 6 errors** in unittest discovery: missing Nav2 modules/resources prevent remaining tests from running |
| Webots launch | **Fails** at launch import: `No module named 'webots_ros2_driver'`; the configured `/usr/local/webots/webots` executable is also absent |
| Robot appears | **Could not test**; Webots did not start |
| ROS node communication | **Could not test** for the simulation; no runtime graph started |
| Camera publication/data | **Could not test**; configuration exists but no frames were received/tested |
| Movement and stop | **Could not test**; no live command sent |
| Simulation/ROS clock agreement | **Could not test**; no live clock available |
| Launcher and test entry points | **Verified failure path**: both exit 3 on the missing Nav2 build dependency |
| Added shell/Python code | `bash -n`, Python compilation and `git diff --check` succeed; live probe and successful-launch cleanup paths remain untested |

Missing ROS package resources found: `webots_ros2_driver`, `webots_ros2_control`,
`nav2_msgs`, `nav2_bringup`, `slam_toolbox`, `laser_filters`. Installed resources
include `controller_manager`, `diff_drive_controller`, `joint_state_broadcaster`,
`robot_state_publisher`, `tf2_ros`, `rviz2` and `rqt_image_view`.

## Exact commands run

Commands below were run from the repository root. The direct launch and initial
Python tests used the inherited DDS domain 42. No integration nodes started:
Nav2-dependent modules failed before those tests could execute.

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base /tmp/tiago-validation/log build --base-paths . \
  --build-base /tmp/tiago-validation/build \
  --install-base /tmp/tiago-validation/install --symlink-install
```

Initial result: two packages finished, `explore_lite` failed and
`simulation_actions` was aborted. Relevant error:

```text
CMake Error at CMakeLists.txt:42 (find_package):
  By not providing "Findnav2_msgs.cmake" in CMAKE_MODULE_PATH this project
  has asked CMake to find a package configuration file provided by
  "nav2_msgs", but CMake did not find one.
  nav2_msgsConfig.cmake
  nav2_msgs-config.cmake
```

```bash
source /opt/ros/jazzy/setup.bash
ROS_LOG_DIR=/tmp/tiago-validation/ros-log timeout --signal=INT 25s \
  ros2 launch ./launch/tiago_apartment_ros2.launch.py
```

Exit 1, before Webots started:

```text
Caught multiple exceptions when trying to load file of format [py]:
 - ModuleNotFoundError: No module named 'webots_ros2_driver'
 - InvalidFrontendLaunchFileError: The launch file may have a syntax error, or its format is unknown
```

The frontend error accompanies the import failure; it does not prove a launch
syntax defect. `ls -l /usr/local/webots/webots` independently returned
`No such file or directory`; `command -v webots` found no executable.

```bash
source /opt/ros/jazzy/setup.bash
python3 -m unittest discover -s tests -p 'test_*.py' -v \
  > /tmp/tiago-validation/python-tests.log 2>&1
```

Exit 1: five map-alignment and four door-preparation tests pass. Explorer,
finalizer and progress test modules fail to import `nav2_msgs`. Three navigation
launch tests error on missing `nav2_bt_navigator`, `nav2_common`, or
`nav2_bringup`. Unittest reports `Ran 15 tests`, `FAILED (errors=6)`; failed
imports are discovery placeholders, not six executed functional tests.

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base /tmp/tiago-validation/log build --base-paths . \
  --build-base /tmp/tiago-validation/build \
  --install-base /tmp/tiago-validation/install --symlink-install \
  --packages-select simulation_actions navigate_to_position explore_lite_msgs
colcon --log-base /tmp/tiago-validation/log test \
  --build-base /tmp/tiago-validation/build \
  --install-base /tmp/tiago-validation/install \
  --packages-select simulation_actions navigate_to_position explore_lite_msgs
colcon test-result --test-result-base /tmp/tiago-validation/build --verbose
```

All commands exit 0; all three selected packages build. Test result:
`Summary: 0 tests, 0 errors, 0 failures, 0 skipped`.

```bash
SIM_WORKSPACE=/tmp/tiago-validation ./run_simulation.sh \
  > /tmp/tiago-validation/launcher.log 2>&1
SIM_WORKSPACE=/tmp/tiago-validation ./test.sh \
  > /tmp/tiago-validation/test-script.log 2>&1
bash -n run_simulation.sh test.sh
python3 -m py_compile scripts/check_simulation.py
git diff --check
```

Both entry points exit 3: three packages finish, then the explorer build fails on
`nav2_msgs`. `test.sh` consequently does not reach colcon/Python tests or start
the runtime probe. Static checks exit 0. No live validation is inferred from
these static checks. After installing prerequisites, rerun `./test.sh`; its log
is `log/simulation-test.log` (or `$SIM_WORKSPACE/log/simulation-test.log`).

## Decisions and remaining limits

Changes add entry points, a live probe and documentation only; existing worlds,
launch files, controller configuration and action semantics are unchanged.
The launcher defaults to the existing full-robot launch, making cameras and
manual control available without Nav2 issuing motion. `--navigation` opts into
the existing navigation launch. The full build intentionally includes the
vendored explorer, even for manual simulation, so missing dependencies are visible.

The live test isolates DDS discovery and checks the shared TCP port before
starting Webots. Only its own launched processes are signaled. DDS discovery
checks cannot prevent another process from subsequently joining that domain;
reserve the chosen test domain while running tests. The live probe's small
rotation is a smoke test, not proof of navigation or manipulation correctness.
Visual appearance, full Nav2 operation and every object action still require
a follow-up graphics-enabled run. Previous results in
`tests/MAPPING_VERIFICATION.md` have not been reverified on this host.
