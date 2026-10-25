"""The GLIM process: start it in its own process group, stop it so it saves its map.

glim_rosnode finishes on SIGINT/SIGTERM the same way: rclcpp shuts down, spin() returns, the node
joins odometry -> local -> global mapping (a final optimization) and serializes the map to
`dump_path`. That can take minutes after a long session, so stopping is a budget, not a wait.
"""
import logging
import os
import signal
import subprocess
import sys
import threading
import time

LOG = logging.getLogger("glim-supervisor")

SIGTERM_GRACE_S = 10
SIGKILL_GRACE_S = 5


class GlimProcess:
    """`on_line`, when set before start(), sees every line GLIM writes (stdout and stderr, merged).
    The lines are forwarded to the supervisor's stderr unchanged, so the container log is the same
    as if GLIM wrote it directly."""

    def __init__(self, command, *, clock=time.monotonic, on_line=None):
        self.command = list(command)
        self.clock = clock
        self.on_line = on_line
        self.popen = None
        self.started_at = None
        self.reader = None

    @property
    def pid(self):
        return self.popen.pid if self.popen else None

    def start(self):
        if self.popen is not None:
            raise RuntimeError("GLIM process already started")
        # Own session: container signals reach only the supervisor, which decides how GLIM stops.
        self.popen = subprocess.Popen(self.command, start_new_session=True,
                                      stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        self.started_at = self.clock()
        self.reader = threading.Thread(target=self._pump, name="glim-log", daemon=True)
        self.reader.start()
        LOG.info("started GLIM (pid %d): %s", self.popen.pid, " ".join(self.command))

    def _pump(self):
        # Never let a slow or failing consumer back-pressure GLIM: forward first, classify after,
        # and swallow consumer errors. spdlog flushes per message, so lines arrive promptly.
        sink = getattr(sys.stderr, "buffer", None)
        for raw in iter(self.popen.stdout.readline, b""):
            try:
                if sink is not None:
                    sink.write(raw)
                    sink.flush()
            except (OSError, ValueError):
                pass
            if self.on_line is not None:
                try:
                    self.on_line(raw.decode("utf-8", "replace"))
                except Exception:
                    pass
        self.popen.stdout.close()

    def running(self):
        return self.popen is not None and self.popen.poll() is None

    def returncode(self):
        return None if self.popen is None else self.popen.poll()

    def uptime(self):
        return 0.0 if self.started_at is None else self.clock() - self.started_at

    def stop(self, save_timeout_s):
        """SIGINT and wait up to `save_timeout_s` for the map save, then escalate.

        Returns (outcome, exit code): "saved" (clean exit inside the budget), "not_started"
        (interrupted before it initialized: nothing to save), "exited" (it was already gone),
        "save_timeout" (needed SIGTERM/SIGKILL; the dump may be incomplete) or "failed" (non-zero
        exit).
        """
        if self.popen is None:
            return "exited", None
        if self.popen.poll() is not None:
            return "exited", self.popen.returncode
        started = self.clock()
        outcome = "saved"
        for sig, budget in ((signal.SIGINT, save_timeout_s), (signal.SIGTERM, SIGTERM_GRACE_S),
                            (signal.SIGKILL, SIGKILL_GRACE_S)):
            if sig != signal.SIGINT:
                LOG.warning("GLIM still running after %.0fs; sending %s", self.clock() - started,
                            sig.name)
                outcome = "save_timeout"
            try:
                os.killpg(self.popen.pid, sig)
            except ProcessLookupError:
                break
            try:
                self.popen.wait(timeout=budget)
                break
            except subprocess.TimeoutExpired:
                continue
        try:
            code = self.popen.wait(timeout=1)
        except subprocess.TimeoutExpired:  # unkillable (uninterruptible I/O): report, don't hang
            LOG.error("GLIM (pid %d) survived SIGKILL", self.popen.pid)
            return "save_timeout", None
        if self.reader is not None:
            self.reader.join(timeout=2)   # the last lines (the save) are in the log before we report
        if outcome == "saved" and code == -signal.SIGINT:
            # Killed by SIGINT's default action: rclcpp had not installed its handler yet, so GLIM
            # never finished initializing and no map can exist (seen within ~1 s of start under
            # rmw_zenoh). Not a failed save.
            outcome = "not_started"
        elif outcome == "saved" and code != 0:
            outcome = "failed"
        LOG.info("GLIM stopped (%s, exit %s) in %.1fs", outcome, code, self.clock() - started)
        return outcome, code
