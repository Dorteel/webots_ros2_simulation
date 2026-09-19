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
