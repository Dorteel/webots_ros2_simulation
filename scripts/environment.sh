#!/usr/bin/env bash
# Shared profiles: only source paths live here; dependencies live in package.xml.
simulation_environment() {
    if [[ ${ROS_DISTRO:-jazzy} != jazzy ]]; then
        echo "Unsupported ROS_DISTRO=${ROS_DISTRO}; use a clean ROS 2 Jazzy shell." >&2
        return 2
    fi
    local setup=${ROS_SETUP:-/opt/ros/jazzy/setup.bash}
    [[ -r $setup ]] || { echo "Missing ROS setup: $setup. Install ROS 2 Jazzy or set ROS_SETUP." >&2; return 2; }
    source "$setup"
    [[ ${ROS_VERSION:-} == 2 && ${ROS_DISTRO:-} == jazzy ]] || {
        echo 'The selected setup must provide ROS 2 Jazzy.' >&2; return 2;
    }
    for command in ros2 rosdep colcon python3 timeout setsid cmake cc c++ make; do
        command -v "$command" >/dev/null || {
            echo "Missing $command. See README.md fresh installation instructions." >&2; return 2;
        }
    done
}

simulation_profile() {
    PROFILE=${1:-basic}
    SOURCE_PATHS=("$ROOT/simulation_actions" "$ROOT/dependencies/basic")
    case "$PROFILE" in
        basic) ;;
        generic|multi-robot) SOURCE_PATHS=("$ROOT/dependencies/generic") ;;
        navigation|mapping)
            SOURCE_PATHS+=("$ROOT/navigate_to_position" "$ROOT/dependencies/navigation") ;;
        *) echo "Unknown profile: $PROFILE (basic, generic, multi-robot, navigation, mapping)" >&2; return 2 ;;
    esac
    if [[ $PROFILE == mapping ]]; then
        SOURCE_PATHS+=("$ROOT/third_party/explore" "$ROOT/third_party/explore_lite_msgs" "$ROOT/dependencies/mapping")
    fi
}

simulation_dependencies() {
    echo "Checking $PROFILE dependencies from package.xml files..."
    if ! rosdep check --from-paths "${SOURCE_PATHS[@]}" --ignore-src --rosdistro "$ROS_DISTRO"; then
        echo "Prerequisites unavailable. Run ./install-dependencies.sh --profile $PROFILE" >&2
        echo 'If rosdep has no database, run sudo rosdep init, then rosdep update.' >&2
        return 2
    fi
    # Discovery must work before creating any build directories.
    colcon list --base-paths "${SOURCE_PATHS[@]}" || return 2
}
