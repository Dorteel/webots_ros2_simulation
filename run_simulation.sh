#!/usr/bin/env bash
# File-based launch files stay the single source of simulation configuration.
set -eo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
WORKSPACE=${SIM_WORKSPACE:-$ROOT}
source "$ROOT/scripts/environment.sh"
PROFILE=basic
BUILD_ONLY=false
CHECK_ONLY=false
LAUNCH_ARGUMENTS=()
while (($#)); do
    case "$1" in
        --navigation) PROFILE=navigation; shift ;;
        --profile) [[ $# -ge 2 ]] || exit 2; PROFILE=$2; shift 2 ;;
        --build-only) BUILD_ONLY=true; shift ;;
        --check-only) CHECK_ONLY=true; shift ;;
        --help) echo 'Usage: ./run_simulation.sh [--navigation | --profile basic|navigation|mapping] [--check-only | --build-only] [name:=value ...]'; exit 0 ;;
        *:=*) LAUNCH_ARGUMENTS+=("$1"); shift ;;
        *) echo "Unknown option: $1" >&2; exit 2 ;;
    esac
done
simulation_environment
simulation_profile "$PROFILE"
prerequisites=0
simulation_dependencies || prerequisites=2
python3 "$ROOT/scripts/preflight.py" "$PROFILE" "${LAUNCH_ARGUMENTS[@]}" || prerequisites=2
[[ $prerequisites == 0 ]] || exit 2
$CHECK_ONLY && exit 0
mkdir -p "$WORKSPACE"
cd "$WORKSPACE"
# Incremental colcon builds also detect changed sources and interfaces.
if ! timeout --signal=INT --kill-after=10s 600s colcon --log-base "$WORKSPACE/log" build \
    --base-paths "${SOURCE_PATHS[@]}" --build-base "$WORKSPACE/build" \
    --install-base "$WORKSPACE/install" --symlink-install; then
    echo 'Build failed; inspect log/latest_build. See REPOSITORY_GUIDE.md for dependencies.' >&2
    exit 3
fi
source "$WORKSPACE/install/setup.bash"
$BUILD_ONLY && exit 0
python3 - <<'PY' || exit 2
import socket
with socket.socket() as probe:
    try:
        probe.bind(('127.0.0.1', 8765))
    except OSError:
        raise SystemExit('Supervisor port 8765 is occupied; stop the other apartment simulation first.')
PY
export ROS_LOG_DIR=${ROS_LOG_DIR:-$WORKSPACE/log/ros}
mkdir -p "$ROS_LOG_DIR"
cd "$ROOT"
cat <<EOF
Starting full TIAGo apartment (ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}). Ctrl-C stops this launch.
In another terminal:
  source /opt/ros/jazzy/setup.bash
  source "$WORKSPACE/install/setup.bash"
  export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
  ros2 topic list
  ros2 topic hz /tiago/camera/color/image_raw
  ros2 run rqt_image_view rqt_image_view
  python3 "$ROOT/arrow_teleop.py"
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped '{}'
Optional navigation: ./run_simulation.sh --navigation
EOF
LAUNCH=tiago_apartment_ros2.launch.py
if [[ $PROFILE == navigation ]]; then LAUNCH=navigation.launch.py; fi
if [[ $PROFILE == mapping ]]; then LAUNCH=tiago_apartment_mapping.launch.py; fi
child=
cleanup() {
    trap '' INT TERM
    if [[ -n $child ]]; then
        kill -INT -- "-$child" 2>/dev/null || true
        for ((i=0; i<100; i++)); do
            kill -0 -- "-$child" 2>/dev/null || break
            sleep 0.1
        done
        kill -TERM -- "-$child" 2>/dev/null || true
        sleep 1
        kill -KILL -- "-$child" 2>/dev/null || true
        wait "$child" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
setsid ros2 launch "$ROOT/launch/$LAUNCH" "${LAUNCH_ARGUMENTS[@]}" &
child=$!
wait "$child"
