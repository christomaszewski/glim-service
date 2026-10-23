"""python3 -m glim_supervisor serve|state|standby|activate|health

`serve` is the container's main process. The other commands are clients of its control socket:
`state` prints exactly one JSON object on stdout (the rig contract); `standby`/`activate` block
until the transition completes and exit non-zero with the reason on failure; `health` is the
Docker healthcheck (exit 0 = healthy).
"""
import fcntl
import json
import logging
import os
from pathlib import Path
import signal
import sys
import threading
import time

from . import control

LOG = logging.getLogger("glim-supervisor")

INSTANCE_FILE = os.environ.get("GLIM_INSTANCE_FILE", "/etc/glim/instance.json")
TARGET_FILE = os.environ.get("GLIM_TARGET_FILE", "/var/lib/glim/target-state")
TARGET_SEED = os.environ.get("GLIM_TARGET_SEED", "/etc/glim/target")
DEFAULTS_DIR = os.environ.get("GLIM_DEFAULTS_DIR", "/opt/glim/share/glim/config")
UPSTREAM_SOURCES = os.environ.get("GLIM_UPSTREAM_SOURCES", "/opt/glim/share/upstream-sources.json")
GLIM_BINARY = os.environ.get("GLIM_ROSNODE", "/opt/glim/lib/glim_ros/glim_rosnode")


def load_instance(path=INSTANCE_FILE):
    with open(path) as stream:
        instance = json.load(stream)
    if instance.get("version") != 1:
        raise ValueError(f"{path}: unsupported instance schema {instance.get('version')!r} — "
                         "the launcher and image are from different releases")
    return instance


def initial_target():
    """The target a NEW container starts toward (a restarted one resumes its persisted target).

    The launcher writes it to a mounted file rather than the environment: Compose recreates a
    container whose environment changed, so an env-carried target would make a plain `up` after
    `rig activate` replace a running mapping session.
    """
    try:
        target = Path(TARGET_SEED).read_text().strip()
    except OSError:
        target = os.environ.get("GLIM_TARGET_STATE", "active")
    if target not in ("active", "standby"):
        raise ValueError(f"initial target {target!r} must be active or standby")
    return target


class Supervisor:
    """What the control socket talks to: transitions go through the controller; the supervise
    loop then carries them into the ROS lifecycle state."""

    def __init__(self, controller):
        self.controller = controller

    def request(self, target):
        self.controller.transition(target)


def serve():
    from .controller import Controller, load_provenance, service_identity

    # The handlers only record the request. Event.set() takes a lock the main thread may already
    # hold inside Event.wait(), so calling it from a signal handler can deadlock the supervisor —
    # and a hung supervisor is SIGKILLed with the map unsaved.
    stop_requests = []
    signal.signal(signal.SIGTERM, lambda signum, _: stop_requests.append(signum))
    signal.signal(signal.SIGINT, lambda signum, _: stop_requests.append(signum))
    stop_event = threading.Event()
    instance = load_instance()
    map_root = Path(os.environ.get("GLIM_MAP_ROOT", "/var/lib/glim/store"))
    initial = initial_target()
    controller = Controller(
        instance, defaults_dir=DEFAULTS_DIR, map_root=map_root, target_file=TARGET_FILE,
        initial=initial, glim_binary=GLIM_BINARY,
        provenance={**load_provenance(UPSTREAM_SOURCES), **service_identity()},
        stop_event=stop_event)
    identity = {"vehicle_id": os.environ.get("VEHICLE_ID", ""), "instance": instance["instance"],
                "service": instance["service"]}

    # ROS starts in the background: rmw_zenoh can block in init until the router is reachable, and
    # the control plane (state/standby/activate/health) must never depend on the ROS graph.
    ros_holder = []

    def start_ros():
        try:
            from .ros_iface import RosInterface
            ros = RosInterface(controller, instance, identity, map_root)
            controller.ready_probe = ros.glim_visible
            ros.start()
            ros_holder.append(ros)
            LOG.info("ROS interface up: /diagnostics, lifecycle node %s/glim_supervisor",
                     instance["namespace"])
        except Exception as error:  # keep mapping; only health/lifecycle are lost
            LOG.error("ROS interface unavailable (no /diagnostics, no lifecycle node): %s", error)

    threading.Thread(target=start_ros, name="ros-init", daemon=True).start()

    socket_path = Path(control.SOCKET)
    socket_path.parent.mkdir(parents=True, exist_ok=True)
    clean = False
    with socket_path.with_suffix(".lock").open("w") as owner:
        fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)  # one supervisor per container
        socket_path.unlink(missing_ok=True)
        with control.Server(str(socket_path), control.Handler) as server:
            os.chmod(socket_path, 0o600)
            server.supervisor = Supervisor(controller)
            thread = threading.Thread(target=server.serve_forever, name="control", daemon=True)
            thread.start()
            LOG.info("supervising %s (namespace %s, pipeline %s, target %s, maps under %s)",
                     instance["instance"], instance["namespace"], instance["pipeline"],
                     controller.target, map_root)
            try:
                while not stop_requests:
                    controller.tick()
                    for ros in ros_holder:
                        ros.sync()
                    time.sleep(1.0)
            finally:
                LOG.info("stopping: saving the running session's map (budget %.0fs)",
                         controller.save_timeout_s)
                clean = controller.shutdown()
                server.shutdown()
                thread.join(timeout=5)
                for ros in ros_holder:
                    ros.close()
                socket_path.unlink(missing_ok=True)
    LOG.info("stopped%s", "" if clean else " — the last map save did not complete")
    return 0 if clean else 1


def client(command):
    if command in ("standby", "activate"):
        # A standby saves the running map; allow the configured budget plus margin.
        try:
            budget = float(load_instance()["map"]["save_timeout_s"]) + 120
        except (OSError, ValueError, KeyError):
            budget = 600.0
    else:
        budget = 15.0
    result = control.request(command, timeout=budget)
    if command == "state":
        print(json.dumps(result))
        return 0
    if command == "health":
        if not result["healthy"]:
            sys.stderr.write(f"unhealthy: {result['detail']}\n")
        return 0 if result["healthy"] else 1
    return 0


def main(argv=None):
    logging.basicConfig(level=logging.INFO, format="%(name)s: %(message)s", stream=sys.stderr)
    args = sys.argv[1:] if argv is None else argv
    command = args[0] if len(args) == 1 else ""
    try:
        if command == "serve":
            return serve()
        if command in control.COMMANDS:
            return client(command)
        raise ValueError("usage: python3 -m glim_supervisor serve|state|standby|activate|health")
    except Exception as error:
        LOG.error("%s", error)
        return 1


if __name__ == "__main__":
    sys.exit(main())
