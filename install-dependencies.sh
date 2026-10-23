#!/usr/bin/env bash
# Repeatable rosdep install; never maintains a second list of apt packages.
set -eo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$ROOT/scripts/environment.sh"
PROFILE=basic
CHECK=false
while (($#)); do
    case "$1" in
        --profile) [[ $# -ge 2 ]] || exit 2; PROFILE=$2; shift 2 ;;
        --navigation) PROFILE=navigation; shift ;;
        --exploration|--all) PROFILE=mapping; shift ;;
        --check) CHECK=true; shift ;;
        --help) echo 'Usage: ./install-dependencies.sh [--profile basic|navigation|mapping] [--check]'; exit 0 ;;
        *) echo "Unknown option: $1" >&2; exit 2 ;;
    esac
done
simulation_environment
simulation_profile "$PROFILE"
if $CHECK; then simulation_dependencies; exit; fi
if [[ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    echo 'rosdep is not initialized. Run: sudo rosdep init' >&2; exit 2
fi
rosdep update --rosdistro "$ROS_DISTRO" || {
    echo 'rosdep update failed. Check network/proxy access; no dependencies were installed.' >&2; exit 2;
}
rosdep install --from-paths "${SOURCE_PATHS[@]}" --ignore-src --rosdistro "$ROS_DISTRO" || {
    echo 'rosdep installation failed. Check the reported apt/rosdep errors and retry; partial installs are safe to retry.' >&2; exit 3;
}
simulation_dependencies
