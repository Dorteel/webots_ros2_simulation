"""
Contents
--------
cleanup()       - Returns the ordered apartment cleanup actions.
run_sequence()  - Executes actions in order and stops on failure.
parse_arguments() - Parses the sequence name.
main()          - Runs a predefined action sequence.
"""

import argparse
import json
import time

from action_cli import send_action


ROBOT = "TIAGo++"
ROUND_TABLE = "round table"
SINK = "sink(1)"
FRIDGE = "fridge(1)"
LOWER_FRIDGE_DOOR = "lower fridge door"
FRIDGE_LOWER_SHELF = [-0.52, -0.50, 0.55]


def cleanup():
    """Return the ordered cleanup actions; edit this list to add more objects."""
    return [
        ("move_to_object", {"robot": ROBOT, "object": ROUND_TABLE}, 1.0),
        ("pick", {"robot": ROBOT, "object": "plate(10)"}, 1.0),
        ("move_to_object", {"robot": ROBOT, "object": SINK}, 1.0),
        ("place_to_object", {"robot": ROBOT, "object": "plate(10)", "target": SINK}, 1.0),
        ("move_to_object", {"robot": ROBOT, "object": ROUND_TABLE}, 1.0),
        ("pick", {"robot": ROBOT, "object": "plate(9)"}, 1.0),
        ("move_to_object", {"robot": ROBOT, "object": SINK}, 1.0),
        ("place_to_object", {"robot": ROBOT, "object": "plate(9)", "target": SINK}, 1.0),
        ("move_to_object", {"robot": ROBOT, "object": "jam jar"}, 1.0),
        ("pick", {"robot": ROBOT, "object": "jam jar"}, 1.0),
        ("move_to_object", {"robot": ROBOT, "object": FRIDGE}, 1.0),
        ("open", {"robot": ROBOT, "object": LOWER_FRIDGE_DOOR}, 2.0),
        ("place", {"robot": ROBOT, "object": "jam jar", "coordinates": FRIDGE_LOWER_SHELF}, 1.0),
        ("close", {"robot": ROBOT, "object": LOWER_FRIDGE_DOOR}, 2.0),
        ("move_to_object", {"robot": ROBOT, "object": ROUND_TABLE}, 1.0),
    ]


def run_sequence(actions):
    """Execute actions in order, raising an error at the first failure."""
    total = len(actions)
    for index, (action, parameters, wait_seconds) in enumerate(actions, start=1):
        if wait_seconds < 0:
            raise ValueError("action wait time must be non-negative")
        print(
            f"[{index}/{total}] {action} {json.dumps(parameters, sort_keys=True)} "
            f"(wait {wait_seconds:g}s)"
        )
        response = send_action(action, parameters)
        if not response.get("ok"):
            raise RuntimeError(response.get("error", "unknown Supervisor error"))
        if "result" in response:
            print(json.dumps(response["result"], sort_keys=True))
        time.sleep(wait_seconds)


def parse_arguments():
    """Parse the name of the predefined sequence to execute."""
    parser = argparse.ArgumentParser(description="Run ordered Webots fallback actions")
    parser.add_argument("sequence", choices=("cleanup",))
    return parser.parse_args()


def main():
    """Run the selected sequence against the active Supervisor."""
    arguments = parse_arguments()
    sequences = {"cleanup": cleanup}
    try:
        run_sequence(sequences[arguments.sequence]())
    except (OSError, RuntimeError, ValueError) as error:
        raise SystemExit(f"error: {error}") from None
    print("done")


if __name__ == "__main__":
    main()
