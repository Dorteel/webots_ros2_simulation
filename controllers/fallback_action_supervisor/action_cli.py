"""
Contents
--------
parse_arguments() - Parses a fallback action command.
make_parameters() - Converts command arguments to action parameters.
send_action()     - Sends an action to the running Webots Supervisor.
main()            - Runs the command-line client.
"""

import argparse
import json
import socket

from command_server import HOST, PORT


def parse_arguments():
    """Parse the compact positional syntax for all fallback actions."""
    parser = argparse.ArgumentParser(description="Call a Webots fallback action")
    parser.add_argument(
        "action", choices=("move", "move_to_object", "pick", "place", "place_to_object", "open", "close")
    )
    parser.add_argument("robot", help="robot name")
    parser.add_argument("object", nargs="?", help="object name")
    parser.add_argument("target", nargs="?", help="target object name")
    parser.add_argument("coordinates", nargs="*", type=float, metavar="COORDINATE")
    parser.add_argument("--distance", type=float, default=0.8, help="stand-off distance (default: 0.8)")
    parser.add_argument("--clearance", type=float, default=0.15, help="extra obstacle clearance (default: 0.15)")
    return parser.parse_args()


def make_parameters(arguments):
    """Build and validate an action parameter dictionary."""
    parameters = {"robot": arguments.robot}
    if arguments.action == "move":
        if arguments.object is None:
            raise ValueError("move requires: ROBOT X Y Z")
        coordinates = [arguments.object]
        if arguments.target is not None:
            coordinates.append(arguments.target)
        coordinates.extend(arguments.coordinates)
        try:
            parameters["coordinates"] = [float(value) for value in coordinates]
        except ValueError:
            raise ValueError("move coordinates must be numbers") from None
    else:
        if arguments.object is None:
            raise ValueError(f"{arguments.action} requires an object name")
        parameters["object"] = arguments.object
        if arguments.action == "move_to_object":
            if arguments.target is not None or arguments.coordinates:
                raise ValueError("move_to_object does not accept coordinates")
            parameters["distance"] = arguments.distance
            parameters["clearance"] = arguments.clearance
        elif arguments.action == "place_to_object":
            if arguments.target is None:
                raise ValueError("place_to_object requires a target object name")
            if arguments.coordinates:
                raise ValueError("place_to_object does not accept coordinates")
            parameters["target"] = arguments.target
        elif arguments.action == "place":
            coordinates = [] if arguments.target is None else [arguments.target]
            parameters["coordinates"] = [*coordinates, *arguments.coordinates]
        elif arguments.target is not None or arguments.coordinates:
            raise ValueError(f"{arguments.action} does not accept coordinates")
    return parameters


def send_action(action, parameters):
    """Send one request and return the Supervisor response."""
    request = json.dumps({"action": action, "parameters": parameters}) + "\n"
    with socket.create_connection((HOST, PORT), timeout=3) as connection:
        connection.sendall(request.encode("utf-8"))
        return json.loads(connection.recv(65536).decode("utf-8"))


def main():
    """Send the requested action and print its result."""
    arguments = parse_arguments()
    try:
        parameters = make_parameters(arguments)
        response = send_action(arguments.action, parameters)
    except (ValueError, OSError) as error:
        raise SystemExit(f"error: {error}") from None
    if not response.get("ok"):
        raise SystemExit(f"error: {response.get('error', 'unknown Supervisor error')}")
    print("ok")
    if "result" in response:
        print(json.dumps(response["result"], sort_keys=True))


if __name__ == "__main__":
    main()
