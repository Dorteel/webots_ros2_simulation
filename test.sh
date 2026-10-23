#!/usr/bin/env bash
# Use an otherwise unused DDS domain; never kill processes by name.
set -eo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export SIM_WORKSPACE=${SIM_WORKSPACE:-$ROOT}
export ROS_DOMAIN_ID=${SIM_TEST_DOMAIN_ID:-87}
PROFILE=basic
case "${1:-}" in
    --all) PROFILE=mapping ;;
    '') ;;
    --help) echo 'Usage: ./test.sh [--all] (all includes navigation/mapping regressions)'; exit 0 ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
esac
# Exit 2 means unavailable prerequisites; no build or simulation is attempted.
"$ROOT/run_simulation.sh" --profile "$PROFILE" --check-only
"$ROOT/run_simulation.sh" --profile "$PROFILE" --build-only
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
source "$SIM_WORKSPACE/install/setup.bash"
source "$ROOT/scripts/environment.sh"
simulation_profile "$PROFILE"
export ROS_LOG_DIR="$SIM_WORKSPACE/log/test-ros"
mkdir -p "$ROS_LOG_DIR"
timeout --signal=INT --kill-after=5s 15s python3 - <<'PY' || exit 2
import time
import rclpy
rclpy.init()
node = rclpy.create_node('simulation_test_domain_check')
deadline = time.monotonic() + 3
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
others = [n for n, ns in node.get_node_names_and_namespaces()
          if n != node.get_name()]
node.destroy_node()
rclpy.shutdown()
if others:
    raise SystemExit('Test domain is occupied: ' + ', '.join(others))
PY
timeout --signal=INT --kill-after=10s 180s colcon --log-base "$SIM_WORKSPACE/log" test \
    --base-paths "${SOURCE_PATHS[@]}" \
    --build-base "$SIM_WORKSPACE/build" --install-base "$SIM_WORKSPACE/install" || exit 4
while read -r package; do
    colcon test-result --test-result-base "$SIM_WORKSPACE/build/$package" --verbose || exit 4
done < <(colcon list --base-paths "${SOURCE_PATHS[@]}" --names-only)
if [[ $PROFILE == mapping ]]; then
    timeout --signal=INT --kill-after=10s 300s python3 -m unittest discover -s "$ROOT/tests" -p 'test_*.py' -v || exit 4
else
    # Basic mode never imports or builds Nav2/exploration tests.
    for module in test_environment.py test_map_alignment.py test_mapping_doors.py test_look_turn.py; do
        timeout --signal=INT --kill-after=10s 60s python3 -m unittest discover -s "$ROOT/tests" -p "$module" -v || exit 4
    done
fi
child=
cleanup() {
    trap '' INT TERM
    if [[ -n $child ]]; then
        # Signal our wrapper; it owns and cleans its dedicated launch group.
        kill -TERM "$child" 2>/dev/null || true
        result=0
        wait "$child" 2>/dev/null || result=$?
        if [[ $result != 0 && $result != 143 && $result != 130 ]]; then
            echo "Launch wrapper shutdown returned $result" >&2
            return 1
        fi
        child=''
        echo 'PASS: owned launch processes shut down'
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
"$ROOT/run_simulation.sh" > "$SIM_WORKSPACE/log/simulation-test.log" 2>&1 &
child=$!
timeout --signal=INT --kill-after=10s 150s python3 "$ROOT/scripts/check_simulation.py" || exit 5
timeout --signal=INT --kill-after=10s 150s python3 "$ROOT/scripts/check_look_turn.py" || exit 5
kill -0 "$child" || exit 5
cleanup || exit 5
trap - EXIT
echo 'PASS: build, selected tests, basic simulation, camera/control/time checks and shutdown.'
