"""
Contents
--------
open_server()      - Opens the local fallback-action command socket.
process_commands() - Accepts and executes pending action requests.
close_server()     - Closes the command socket.
"""

import errno
import subprocess
import json
import select
import socket


HOST = "127.0.0.1"
PORT = 8765


def _occupied_message():
    owner = "listener owner unavailable"
    try:
        result = subprocess.run(
            ['ss', '-H', '-ltnp', f'sport = :{PORT}'],
            capture_output=True, text=True, timeout=2, check=False)
        if result.returncode == 0 and result.stdout.strip():
            owner = result.stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        pass
    return (f"Fallback action TCP port {HOST}:{PORT} is already occupied. "
            f"Owner: {owner}. Stop the owning demo/simulator before starting another; "
            "no existing process has been terminated.")


def _bound_socket():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((HOST, PORT))
    except OSError as error:
        server.close()
        if error.errno == errno.EADDRINUSE:
            raise RuntimeError(_occupied_message()) from None
        raise
    return server


def check_port_available():
    """Fail before launching Webots if another simulator/application owns the port.

    This is a preflight, not a reservation; open_server checks again at bind time.
    """
    with _bound_socket():
        pass


def open_server():
    """Open a non-blocking localhost TCP server."""
    server = _bound_socket()
    try:
        server.listen()
        server.setblocking(False)
    except OSError:
        server.close()
        raise
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
