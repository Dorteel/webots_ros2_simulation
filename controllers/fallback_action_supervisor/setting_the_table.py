#!/usr/bin/env python3
"""Run the Webots fallback sequence for setting the table."""

import argparse
import json
import time

from action_cli import send_action


ROBOT = "TIAGo"
CABINET = "cabinet(1)"
FRIDGE = "fridge"
SINK = "sink"

PLATE1 = "plate1"
PLATE2 = "plate2"
CUPCAKE1 = "cupcake1"
CUPCAKE2 = "cupcake2"
FORK1 = "tablefork1"
FORK2 = "tablefork2"
KNIFE1 = "table_knife1"
KNIFE2 = "table_knife2"

ROBOT_POSES = {
    "sink": {
        "coordinates": [-2.6371264927964417, -1.9439168740822361, 0.09060384716557005],
        "rotation": [0.004065782799853591, -0.000728784114064671, 0.9999914691055816, 2.879075110130368],
    },
    "cabinet": {
        "coordinates": [-1.7150399235419191, -1.3534217841017335, 0.09057956211576029],
        "rotation": [0.004766482053587204, -0.00627212018369225, 0.9999689700971895, 1.3079843596604386],
    },
    "fridge": {
        "coordinates": [-0.7357016183530983, -1.4072728798419434, 0.09057740334796638],
        "rotation": [0.0051776976564171595, -0.006044848305185441, 0.9999683251263242, 1.3091103422968604],
    },
    "plate1": {
        "coordinates": [-1.3512134411579946, -2.493935351530973, 0.09056305446646257],
        "rotation": [-0.003681964721424545, -0.0095874603906189, -0.9999472604787957, 1.0461940253032047],
    },
    "plate2": {
        "coordinates": [-2.579725165996911, -3.6280013444038532, 0.09056498697661597],
        "rotation": [0.002657617200302924, -0.03872326508856185, -0.9992464389787425, 0.26050508163652863],
    },
}

PLATE1_POSITION = [-1.02917, -3.11378, 0.765]
PLATE2_POSITION = [-1.86873, -4.06664, 0.765]
KNIFE1_POSITION = [-1.22917, -3.11378, 0.765]
KNIFE2_POSITION = [-1.66873, -4.06664, 0.765]
FORK1_POSITION = [-0.82917, -3.11378, 0.765]
FORK2_POSITION = [-2.6873, -4.06664, 0.765]

MOVE_WAIT = 1.0
ACTION_WAIT = 1.0
DOOR_WAIT = 1.5


def move_to(pose_name):
    """Return a deterministic robot teleport action for a named pose."""
    pose = ROBOT_POSES[pose_name]
    return (
        "move",
        {
            "robot": ROBOT,
            "coordinates": pose["coordinates"],
            "rotation": pose["rotation"],
        },
        MOVE_WAIT,
    )


def setting_the_table():
    """Return the ordered fallback actions for the table-setting demo."""
    return [
        # Plates: cabinet -> final table poses.
        move_to("cabinet"),
        ("open", {"robot": ROBOT, "object": CABINET}, DOOR_WAIT),
        ("pick", {"robot": ROBOT, "object": PLATE1}, ACTION_WAIT),
        ("close", {"robot": ROBOT, "object": CABINET}, DOOR_WAIT),
        move_to("plate1"),
        ("place", {"robot": ROBOT, "object": PLATE1, "coordinates": PLATE1_POSITION}, ACTION_WAIT),

        move_to("cabinet"),
        ("open", {"robot": ROBOT, "object": CABINET}, DOOR_WAIT),
        ("pick", {"robot": ROBOT, "object": PLATE2}, ACTION_WAIT),
        ("close", {"robot": ROBOT, "object": CABINET}, DOOR_WAIT),
        move_to("plate2"),
        ("place", {"robot": ROBOT, "object": PLATE2, "coordinates": PLATE2_POSITION}, ACTION_WAIT),

        # Cupcakes: fridge -> plates.
        move_to("fridge"),
        ("open", {"robot": ROBOT, "object": FRIDGE}, DOOR_WAIT),
        ("pick", {"robot": ROBOT, "object": CUPCAKE1}, ACTION_WAIT),
        ("close", {"robot": ROBOT, "object": FRIDGE}, DOOR_WAIT),
        move_to("plate1"),
        ("place_to_object", {"robot": ROBOT, "object": CUPCAKE1, "target": PLATE1}, ACTION_WAIT),

        move_to("fridge"),
        ("open", {"robot": ROBOT, "object": FRIDGE}, DOOR_WAIT),
        ("pick", {"robot": ROBOT, "object": CUPCAKE2}, ACTION_WAIT),
        ("close", {"robot": ROBOT, "object": FRIDGE}, DOOR_WAIT),
        move_to("plate2"),
        ("place_to_object", {"robot": ROBOT, "object": CUPCAKE2, "target": PLATE2}, ACTION_WAIT),

        # Cutlery: sink -> exact final table coordinates.
        move_to("sink"),
        ("pick", {"robot": ROBOT, "object": FORK1}, ACTION_WAIT),
        move_to("plate1"),
        ("place", {"robot": ROBOT, "object": FORK1, "coordinates": FORK1_POSITION}, ACTION_WAIT),

        move_to("sink"),
        ("pick", {"robot": ROBOT, "object": KNIFE1}, ACTION_WAIT),
        move_to("plate1"),
        ("place", {"robot": ROBOT, "object": KNIFE1, "coordinates": KNIFE1_POSITION}, ACTION_WAIT),

        move_to("sink"),
        ("pick", {"robot": ROBOT, "object": FORK2}, ACTION_WAIT),
        move_to("plate2"),
        ("place", {"robot": ROBOT, "object": FORK2, "coordinates": FORK2_POSITION}, ACTION_WAIT),

        move_to("sink"),
        ("pick", {"robot": ROBOT, "object": KNIFE2}, ACTION_WAIT),
        move_to("plate2"),
        ("place", {"robot": ROBOT, "object": KNIFE2, "coordinates": KNIFE2_POSITION}, ACTION_WAIT),
    ]


def run_sequence(actions):
    """Execute actions in order, raising an error at the first failure."""
    total = len(actions)
    for index, (action, parameters, wait_seconds) in enumerate(actions, start=1):
        if wait_seconds < 0:
            raise ValueError("action wait time must be non-negative")

        print(
            f"[{index}/{total}] {action} "
            f"{json.dumps(parameters, sort_keys=True)} "
            f"(wait {wait_seconds:g}s)"
        )

        response = send_action(action, parameters)
        if not response.get("ok"):
            raise RuntimeError(response.get("error", "unknown Supervisor error"))

        if "result" in response:
            print(json.dumps(response["result"], sort_keys=True))

        time.sleep(wait_seconds)


def parse_arguments():
    """Parse the predefined sequence to execute."""
    parser = argparse.ArgumentParser(description="Run ordered Webots fallback actions")
    parser.add_argument("sequence", choices=("setting_the_table",))
    return parser.parse_args()


def main():
    """Run the selected sequence against the active Webots Supervisor."""
    arguments = parse_arguments()
    sequences = {"setting_the_table": setting_the_table}

    try:
        run_sequence(sequences[arguments.sequence]())
    except (OSError, RuntimeError, ValueError) as error:
        raise SystemExit(f"error: {error}") from None

    print("done")


if __name__ == "__main__":
    main()
