"""
Contents
--------
main() - Starts the single Webots Supervisor loop.
"""

from controller import Supervisor

from actions import execute_action
from command_server import close_server, open_server, process_commands


def main():
    """Keep the shared Supervisor alive for callers of execute_action()."""
    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep())
    server = open_server()

    try:
        while supervisor.step(timestep) != -1:
            process_commands(server, supervisor, execute_action)
    finally:
        close_server(server)


if __name__ == "__main__":
    main()
