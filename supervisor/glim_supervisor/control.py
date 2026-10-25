"""The supervisor's local control socket: one JSON request line, one JSON reply line.

Clients are the launcher's state verbs (`docker compose exec ... glim-statectl <verb>`) and the
container healthcheck. Only root inside the container can connect (mode 0600).
"""
import json
import os
import socket
import socketserver

SOCKET = os.environ.get("GLIM_STATE_SOCKET", "/run/glim/state.sock")
COMMANDS = ("state", "standby", "activate", "health")


class Server(socketserver.ThreadingUnixStreamServer):
    daemon_threads = True
    block_on_close = False


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        self.request.settimeout(5)
        try:
            command = json.loads(self.rfile.readline(1024))["command"]
            supervisor = self.server.supervisor
            if command in ("standby", "activate"):
                supervisor.request("active" if command == "activate" else "standby")
                result = {"ok": True}
            elif command == "state":
                result = {"ok": True, **supervisor.controller.state()}
            elif command == "health":
                healthy, detail = supervisor.controller.health()
                result = {"ok": True, "healthy": healthy, "detail": detail}
            else:
                raise ValueError(f"unknown command: {command}")
        except Exception as error:
            result = {"ok": False, "error": str(error)}
        try:
            self.wfile.write((json.dumps(result) + "\n").encode())
        except (BrokenPipeError, ConnectionResetError):
            pass


def request(command, *, path=None, timeout=15.0):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.settimeout(timeout)
        connection.connect(path or SOCKET)
        connection.sendall((json.dumps({"command": command}) + "\n").encode())
        with connection.makefile("rb") as stream:
            result = json.loads(stream.readline(65536))
    if not result.pop("ok"):
        raise RuntimeError(result["error"])
    return result
