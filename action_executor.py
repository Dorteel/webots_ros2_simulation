#!/usr/bin/env python3
"""Execute the setting-the-table task through the existing ROS 2 actions."""

from time import sleep   # instead of: import asyncio
from pathlib import Path

import rclpy
import yaml
from rclpy.executors import MultiThreadedExecutor

from navigate_to_position.client_api import NavigateToPositionClient
from simulation_actions.pick_client import PickClient
from simulation_actions.place_client import PlaceClient
from simulation_actions.place_to_object_client import PlaceToObjectClient
from simulation_actions.place_in_container_client import PlaceInContainerClient
from simulation_actions.place_next_to_client import PlaceNextToClient
from simulation_actions.open_client import OpenClient
from simulation_actions.close_client import CloseClient


# ---------------------------------------------------------------------------
# Task configuration
# ---------------------------------------------------------------------------

TASK_NAME = "setting_the_table"
WORLD_FILE = "setting_the_table_complete_apartment_tiago_ros2.wbt"
ACTION_DELAY = 2.0
ROBOT = "TIAGo"

NAVIGATION_RETRIES = 3
NAVIGATION_RETRY_DELAY = 1.0

REPO_ROOT = Path(__file__).resolve().parents[0]
SEMANTIC_MAP = REPO_ROOT / "config" / f"{TASK_NAME}_map.yaml"

def load_semantic_map():
    """Load navigation poses and exact Webots placement positions."""
    with SEMANTIC_MAP.open() as file:
        semantic_map = yaml.safe_load(file)

    required = (
        "fridge",
        "cabinet(1)",
        "plate1",
        "plate2",
        "tablefork1",
        "tablefork2",
        "table_knife1",
        "table_knife2",
    )
    missing = [name for name in required if name not in semantic_map]
    if missing:
        raise KeyError(f"Missing semantic-map entries: {', '.join(missing)}")

    return semantic_map


def navigate_to(semantic_map, anchor):
    """Create a Nav2 action from a recorded map-frame approach pose."""
    pose = semantic_map[anchor]["navigation_pose"]
    return {
        "action": "navigate_to_position",
        "x": pose["x"],
        "y": pose["y"],
        "yaw": pose["yaw"],
        "frame_id": pose.get("frame_id", "map"),
    }


def place_at(semantic_map, object_name):
    """Create a fallback `/place` action from a recorded Webots position."""
    position = semantic_map[object_name]["placement_position"]
    return {
        "action": "place",
        "robot": ROBOT,
        "object": object_name,
        "x": position["x"],
        "y": position["y"],
        "z": position["z"],
    }


def create_plan(semantic_map):
    """Create the executable setting-the-table sequence."""
    actions = []

    def nav(anchor):
        actions.append(navigate_to(semantic_map, anchor))

    def pick(obj):
        actions.append({"action": "pick", "robot": ROBOT, "object": obj})

    def open_(obj):
        actions.append({"action": "open", "robot": ROBOT, "object": obj})

    def close(obj):
        actions.append({"action": "close", "robot": ROBOT, "object": obj})

    def place_on(obj, target):
        actions.append({
            "action": "place_to_object",
            "robot": ROBOT,
            "object": obj,
            "target": target,
        })

    def place_exact(obj):
        actions.append(place_at(semantic_map, obj))

    # 1. Take both plates from the cabinet and put them at their table positions.
    nav("cabinet(1)")
    open_("cabinet(1)")
    pick("plate1")
    close("cabinet(1)")
    nav("plate1")
    place_exact("plate1")

    nav("cabinet(1)")
    open_("cabinet(1)")
    pick("plate2")
    close("cabinet(1)")
    nav("plate2")
    place_exact("plate2")

    # 2. Take the cupcakes from the fridge and place them on the plates.
    nav("fridge")
    open_("fridge")
    pick("cupcake1")
    close("fridge")
    nav("plate1")
    place_on("cupcake1", "plate1")

    nav("fridge")
    open_("fridge")
    pick("cupcake2")
    close("fridge")
    nav("plate2")
    place_on("cupcake2", "plate2")

    # 3. Take the cutlery from the cabinet and place it at exact table coordinates.
    #
    # Note: according to the supplied coordinates, tablefork1/table_knife1 are
    # physically beside plate2, while tablefork2/table_knife2 are beside plate1.
    # Fork 1
    nav("sink")
    pick("tablefork1")
    nav("plate1")
    place_exact("tablefork1")

    # Knife 1
    nav("sink")
    pick("table_knife1")
    nav("plate1")
    place_exact("table_knife1")

    # Fork 2
    nav("sink")
    pick("tablefork2")
    nav("plate2")
    place_exact("tablefork2")

    # Knife 2
    nav("sink")
    pick("table_knife2")
    nav("plate2")
    place_exact("table_knife2")

    return actions


class ActionExecutor:
    """Execute existing ROS 2 actions sequentially and fail fast."""

    def __init__(self, node):
        self.clients = {
            "navigate_to_position": NavigateToPositionClient(node).navigate,
            "pick": PickClient(node).pick,
            "place": PlaceClient(node).place,
            "place_to_object": PlaceToObjectClient(node).place_to_object,
            "place_in_container": PlaceInContainerClient(node).place_in_container,
            "place_next_to": PlaceNextToClient(node).place_next_to,
            "open": OpenClient(node).open,
            "close": CloseClient(node).close,
        }

    async def execute(self, actions):
        """Await every action before starting the next one."""
        for index, step in enumerate(actions, start=1):
            parameters = dict(step)
            name = parameters.pop("action")
            client = self.clients.get(name)

            if client is None:
                raise ValueError(f"Unsupported action: {name!r}")

            print(f"[{index}/{len(actions)}] {name}: {parameters}")

            attempts = NAVIGATION_RETRIES if name == "navigate_to_position" else 1

            for attempt in range(attempts):
                success = await client(**parameters)

                if success:
                    break

                if attempt < attempts - 1:
                    print(f"Navigation failed. Retrying ({attempt + 2}/{attempts})...")
                    sleep(NAVIGATION_RETRY_DELAY)

            if not success:
                raise RuntimeError(f"Action {index} ({name}) failed")

            # Non-blocking pause so ROS callbacks can continue to run.
            if ACTION_DELAY > 0:
                sleep(ACTION_DELAY)

        return True


def main():
    semantic_map = load_semantic_map()
    actions = create_plan(semantic_map)

    print(f"Task: {TASK_NAME}")
    print(f"World: {WORLD_FILE}")
    print(f"Semantic map: {SEMANTIC_MAP}")
    print(f"Actions: {len(actions)}")

    rclpy.init()
    node = rclpy.create_node("action_executor")
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        task = executor.create_task(ActionExecutor(node).execute(actions))
        executor.spin_until_future_complete(task)
        task.result()  # Re-raise any exception from the action sequence.
        print("All actions succeeded")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
