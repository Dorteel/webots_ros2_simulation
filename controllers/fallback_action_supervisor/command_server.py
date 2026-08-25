"""
Contents
--------
open_server()      - Opens the local fallback-action command socket.
process_commands() - Accepts and executes pending action requests.
close_server()     - Closes the command socket.
"""

import json
import select
import socket


HOST = "127.0.0.1"
PORT = 8765


def open_server():
    """Open a non-blocking localhost TCP server."""
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen()
    server.setblocking(False)
    print(f"Fallback actions listening on {HOST}:{PORT}")
    return server


def process_commands(server, supervisor, execute_action):
    """Process all connections waiting at the current simulation step."""
    while select.select([server], [], [], 0)[0]:
        connection, _ = server.accept()
        with connection:
            try:
                request = json.loads(connection.recv(65536).decode("utf-8"))
                result = execute_action(
                    supervisor, request.get("action"), request.get("parameters")
                )
                response = {"ok": True}
                if result is not None:
                    response["result"] = result
            except (ValueError, TypeError, UnicodeError, json.JSONDecodeError) as error:
                response = {"ok": False, "error": str(error)}
            connection.sendall((json.dumps(response) + "\n").encode("utf-8"))


def close_server(server):
    """Close the command server."""
    server.close()
