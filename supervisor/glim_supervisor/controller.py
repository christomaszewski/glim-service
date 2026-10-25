"""Operational state for one GLIM instance: the persisted target, mapping sessions, recovery.

active  = target active and a GLIM session is running and visible on the ROS graph
standby = target standby and no session (the last one's map is saved)
anything else is `transitioning` (starting, saving a map, waiting to restart after an exit).

Only the controller starts or stops GLIM. Operator transitions block until the new state is
reached or fail with a reason; the supervise loop's tick() converges everything else (a GLIM
exit while active is restarted with capped exponential backoff, each restart a new session).
"""
import json
import logging
import os
from pathlib import Path
import threading
import time

from . import glimconfig, storage
from .process import GlimProcess

LOG = logging.getLogger("glim-supervisor")

ACTIVE, STANDBY = "active", "standby"
READY_AFTER_S = 2.0       # GLIM must stay up this long before a session counts as started
UNHEALTHY_FAILURES = 3    # consecutive failed sessions before the container reports unhealthy


def stopped_cleanly(outcome, code):
    """Did a stop lose nothing? (GLIM exits 0 after serializing its map; one interrupted before it
    initialized never had a map.)"""
    return outcome in ("saved", "not_started") or (outcome == "exited" and code == 0)


def glim_command(binary, instance, config_dir, dump_dir):
    return [binary, "--ros-args",
            "-r", f"__ns:={instance['namespace']}",
            "-p", f"config_path:={config_dir}",
            "-p", f"dump_path:={dump_dir}",
            "-p", f"use_sim_time:={'true' if instance.get('use_sim_time') else 'false'}"]


class Session:
    def __init__(self, session_id, directory, process, record):
        self.id = session_id
        self.directory = Path(directory)
        self.process = process
        self.record = record


class Controller:
    def __init__(self, instance, *, defaults_dir, map_root, target_file, initial=ACTIVE,
                 glim_binary="glim_rosnode", process_factory=GlimProcess, provenance=None,
                 clock=time.monotonic, wall=time.time, stop_event=None,
                 startup_timeout_s=60.0, stable_after_s=60.0, max_backoff_s=60.0):
        if initial not in (ACTIVE, STANDBY):
            raise ValueError("initial state must be active or standby")
        self.instance = instance
        self.defaults_dir = Path(defaults_dir)
        self.map_root = Path(map_root)
        self.target_file = Path(target_file)
        self.glim_binary = glim_binary
        self.process_factory = process_factory
        self.provenance = provenance or {}
        self.clock = clock
        self.wall = wall
        self.stop_event = stop_event or threading.Event()
        self.startup_timeout_s = startup_timeout_s
        self.stable_after_s = stable_after_s
        self.max_backoff_s = max_backoff_s
        self.save_timeout_s = float(instance["map"]["save_timeout_s"])
        self.ready_probe = None   # optional callable -> bool | None (the ROS graph check)

        self.session_marker = self.target_file.with_name("current-session")
        self.target = initial
        saved = None
        try:
            saved = self.target_file.read_text().strip()
        except FileNotFoundError:
            pass
        except OSError as error:
            LOG.error("cannot read the saved target (%s); starting from %s", error, initial)
        if saved in (ACTIVE, STANDBY):
            self.target = saved
        else:
            if saved is not None:   # e.g. emptied by a power cut: never wedge the container
                LOG.error("ignoring invalid saved target %r; starting from %s", saved, initial)
            # Persist the starting target, so a restart resumes it rather than re-reading a seed
            # the launcher may have rewritten since.
            try:
                self._save_target(initial)
            except OSError as error:
                LOG.error("cannot persist the starting target: %s", error)
        self.lock = threading.Lock()
        self.changing = threading.Event()
        self.stopping = threading.Event()
        self.activity = ""
        self.session = None
        self.last_session = None
        self.error = ""            # why the last session ended unexpectedly / failed to start
        self.transition_error = ""
        self.save_error = ""       # the last map save that did not complete cleanly
        self.unsaved_at_shutdown = False  # a save that failed after the container was told to stop
        self.failures = 0          # consecutive failed sessions; reset once one runs stably
        self.restarts = 0          # unexpected session ends since the supervisor started
        self.next_start_at = 0.0
        self.config_warnings = []

    # --- observation ---------------------------------------------------------------------
    def _ready(self, session):
        if not session.process.running() or session.process.uptime() < READY_AFTER_S:
            return False
        if self.ready_probe is None:
            return True
        try:
            visible = self.ready_probe()
        except Exception:  # a graph query failure must not flip a running session's state
            return True
        return True if visible is None else bool(visible)

    def state(self):
        target, session = self.target, self.session
        if self.stopping.is_set():
            return {"state": "transitioning", "target": target,
                    "detail": "shutting down" + (f": saving map (session {session.id})"
                                                 if session else "")}
        if self.changing.is_set():
            return {"state": "transitioning", "target": target,
                    "detail": self.activity or "transition in progress"}
        if target == STANDBY:
            if session is None:
                last = self.last_session
                detail = f"no session; last {last['id']}: {last['outcome']}" if last else "no session"
                return {"state": STANDBY, "target": target, "detail": detail}
            return {"state": "transitioning", "target": target,
                    "detail": f"stopping session {session.id}"}
        if session is None:
            wait = max(0.0, self.next_start_at - self.clock())
            detail = f"{self.error}; restarting in {wait:.0f}s" if self.error else "starting"
            return {"state": "transitioning", "target": target, "detail": detail}
        if not session.process.running():
            return {"state": "transitioning", "target": target,
                    "detail": f"GLIM exited (code {session.process.returncode()}); restarting"}
        if not self._ready(session):
            return {"state": "transitioning", "target": target,
                    "detail": f"starting GLIM (session {session.id})"}
        return {"state": ACTIVE, "target": target,
                "detail": f"session {session.id}, up {session.process.uptime():.0f}s"}

    def health(self):
        """Container readiness ("can it run"), identical in both states: unhealthy only when
        GLIM keeps failing — a confirmed standby is healthy, and so is a session awaiting data."""
        if self.target == ACTIVE and self.failures >= UNHEALTHY_FAILURES:
            return False, f"{self.failures} consecutive failed sessions: {self.error}"
        return True, "ok"

    def snapshot(self):
        session = self.session
        state = self.state()
        # A transition's error describes the attempt; once the target is reached it is history.
        current_error = self.error or (self.transition_error if state["state"] != self.target else "")
        return {
            **state,
            "restarts": self.restarts,
            "failures": self.failures,
            "error": current_error,
            "save_error": self.save_error,
            "session": session.id if session else "",
            "session_dir": str(session.directory) if session else "",
            "session_uptime_s": session.process.uptime() if session else None,
            "last_session": self.last_session,
            "config_warnings": list(self.config_warnings),
            "pipeline": self.instance.get("pipeline", ""),
        }

    # --- transitions ---------------------------------------------------------------------
    def transition(self, target, revert_on_failure=False):
        """Persist `target` and converge to it; raise with the reason if that fails.

        By default a failed transition keeps the new target (the supervise loop keeps trying,
        like a rig verb). `revert_on_failure` restores the previous target instead — ROS
        lifecycle semantics, where a failed activate leaves the node inactive.
        """
        if target not in (ACTIVE, STANDBY):
            raise ValueError("target must be active or standby")
        # Wait briefly for a tick or probe to finish, but never queue behind another transition.
        if not self.lock.acquire(timeout=20):
            raise RuntimeError("transition in progress; retry when state settles")
        previous = self.target
        try:
            if self.stop_event.is_set():
                raise RuntimeError("supervisor stopping")
            self.changing.set()
            persist_error = None
            try:
                self._save_target(target)
            except OSError as error:
                # Still act on it (a full disk must not keep GLIM from saving its map on standby),
                # but report that a restart would not remember it.
                self.target = target
                persist_error = error
            self.next_start_at = 0.0   # an operator request skips any restart backoff
            self._reconcile(blocking=True)
            if target == ACTIVE and self.session is not None:
                self._await_ready(self.session)   # e.g. a restart begun by tick() moments ago
            if target == STANDBY:
                self.failures = 0                 # parking clears a crash loop's history
                self.error = ""
            if persist_error is not None:
                raise RuntimeError(f"{target} reached, but the target could not be saved for "
                                   f"restarts: {persist_error}")
            self.transition_error = ""
        except Exception as error:
            self.transition_error = str(error)
            if revert_on_failure and previous != target:
                LOG.warning("%s failed; reverting the target to %s", target, previous)
                try:
                    self._save_target(previous)
                    self._reconcile(blocking=False)
                except Exception as revert_error:
                    LOG.error("reverting to %s failed: %s", previous, revert_error)
            raise
        finally:
            self.activity = ""
            self.changing.clear()
            self.lock.release()

    def tick(self):
        """Periodic convergence from the supervise loop; skipped while a transition runs."""
        if not self.lock.acquire(blocking=False):
            return
        try:
            if not self.stop_event.is_set():
                self._reconcile(blocking=False)
        except Exception as error:
            LOG.warning("%s", error)
        finally:
            self.lock.release()

    def shutdown(self):
        """Container stop: save the running session's map. True when nothing was lost."""
        self.stop_event.set()
        self.stopping.set()   # unlike `changing`, no in-flight transition can clear this
        # An in-flight transition may itself be saving a map; give it the same budget.
        if not self.lock.acquire(timeout=self.save_timeout_s + 20):
            LOG.error("shutdown could not acquire the transition lock; map save unconfirmed")
            return False
        try:
            if self.session is not None:
                self._stop_session()
            return not self.unsaved_at_shutdown
        finally:
            self.lock.release()

    # --- internals -----------------------------------------------------------------------
    def _save_target(self, target):
        storage.write_durable(self.target_file, target + "\n")
        self.target = target

    def _pause(self, seconds):
        if self.stop_event.wait(seconds):
            raise RuntimeError("supervisor stopping")

    def _note_failure(self, message):
        self.failures += 1
        self.error = message
        backoff = min(self.max_backoff_s, 2.0 ** min(self.failures, 6))
        self.next_start_at = self.clock() + backoff
        LOG.error("%s (failure %d; next attempt in %.0fs)", message, self.failures, backoff)

    def _reconcile(self, blocking):
        session = self.session
        lost = None
        if session is not None and not session.process.running():
            code = session.process.returncode()
            self.session = None
            self._finish(session, "exited", code)
            if self.target == ACTIVE:
                self.restarts += 1
                self._note_failure(f"GLIM exited unexpectedly (code {code}) in session {session.id}")
            elif not stopped_cleanly("exited", code):
                # It died just before a standby could save it: the map is lost, say so.
                lost = self.save_error = (f"session {session.id}: GLIM exited (code {code}) "
                                          "before its map was saved")
        if self.target == ACTIVE:
            if self.session is None:
                if blocking or self.clock() >= self.next_start_at:
                    self._start_session(wait=blocking)
            elif self.failures and self.session.process.uptime() >= self.stable_after_s:
                LOG.info("session %s stable; clearing %d failure(s)", self.session.id, self.failures)
                self.failures = 0
                self.error = ""
        elif self.session is not None:
            session = self.session
            outcome, code = self._stop_session()
            if not stopped_cleanly(outcome, code):
                raise RuntimeError(f"GLIM stopped but session {session.id}'s map was not saved "
                                   f"cleanly ({outcome}, exit {code}); see the container log")
        if lost and blocking:
            raise RuntimeError(lost)

    def _start_session(self, wait):
        stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime(self.wall()))
        directory = None
        record = {
            "version": 1,
            "instance": self.instance["instance"],
            "pipeline": self.instance.get("pipeline"),
            "inputs": self.instance.get("inputs"),
            "use_sim_time": bool(self.instance.get("use_sim_time")),
            "started_unix": self.wall(),
            "outcome": "running",
            **self.provenance,
        }
        # Everything up to a running process counts as starting: an unwritable or full data dir
        # must become a visible, backed-off failure, not an endless "starting".
        try:
            root = storage.instance_root(self.map_root, self.instance["instance"])
            created = [p for p in (root.parent, root) if not p.exists()]
            directory = storage.new_session_dir(root, stamp)
            for path in created:
                storage.adopt_ownership(path, self.map_root, recursive=False)
            record["session"] = directory.name
            self.activity = f"starting session {directory.name}"
            config_dir, warnings = glimconfig.compose(self.defaults_dir, self.instance, directory)
            self.config_warnings = warnings
            for warning in warnings:
                LOG.warning("%s", warning)
            record["config_warnings"] = warnings
            storage.write_json(directory / "session.json", record)
            process = self.process_factory(glim_command(self.glim_binary, self.instance, config_dir,
                                                        directory / "dump"))
            process.start()
        except Exception as error:
            where = "a new session"
            if directory is not None:
                where = f"session {directory.name}"
                record.update(outcome="failed_to_start", error=str(error), ended_unix=self.wall())
                try:
                    storage.write_json(directory / "session.json", record)
                    storage.adopt_ownership(directory, self.map_root)
                except OSError:
                    pass
            self._note_failure(f"{where} failed to start: {error}")
            raise RuntimeError(self.error) from error
        session = Session(directory.name, directory, process, record)
        self.session = session
        try:   # lets a restarted supervisor close out a session it never got to finish
            storage.write_durable(self.session_marker, str(directory) + "\n")
        except OSError as error:
            LOG.warning("cannot record the running session: %s", error)
        if wait:
            self._await_ready(session)

    def _await_ready(self, session):
        """Block until `session` is up and visible, or fail with the reason."""
        deadline = self.clock() + self.startup_timeout_s
        while self.clock() < deadline:
            if not session.process.running():
                code = session.process.returncode()
                self.session = None
                self._finish(session, "exited", code)
                self._note_failure(f"GLIM exited during startup (code {code}) in session "
                                   f"{session.id}; see the container log")
                raise RuntimeError(self.error)
            if self._ready(session):
                LOG.info("session %s active", session.id)
                return
            self._pause(0.5)
        # Leave it running: the graph check can lag discovery, and the target stays active.
        raise RuntimeError(f"GLIM not visible on the ROS graph {self.startup_timeout_s:.0f}s after "
                           f"starting session {session.id}")

    def recover_interrupted(self):
        """Close out the session a previous run of this supervisor never finished (SIGKILL past
        the grace period, OOM, power loss), so no record claims a session that is not running."""
        try:
            directory = Path(self.session_marker.read_text().strip())
        except OSError:
            return
        try:
            record = json.loads((directory / "session.json").read_text())
            if record.get("outcome") == "running":
                record.update(outcome="interrupted", map_saved=False, ended_unix=None)
                storage.write_json(directory / "session.json", record)
                LOG.warning("session %s was interrupted (supervisor stopped without finishing it)",
                            directory.name)
        except (OSError, ValueError) as error:
            LOG.warning("cannot close out interrupted session %s: %s", directory, error)
        finally:
            self.session_marker.unlink(missing_ok=True)

    def _stop_session(self):
        session = self.session
        self.activity = f"saving map (session {session.id})"
        outcome, code = session.process.stop(self.save_timeout_s)
        self.session = None
        self._finish(session, outcome, code)
        if stopped_cleanly(outcome, code):
            self.save_error = ""
        else:
            self.save_error = f"session {session.id}: {outcome} (exit {code})"
            # A container stop must not exit 0 over a lost map, even when the failing save was a
            # standby that was already in flight when the stop arrived.
            if self.stop_event.is_set():
                self.unsaved_at_shutdown = True
        return outcome, code

    def _finish(self, session, outcome, code):
        dump = session.directory / "dump"
        try:   # a dump cut short by SIGTERM/SIGKILL is not a saved map, however many files it has
            saved = stopped_cleanly(outcome, code) and dump.is_dir() and any(dump.iterdir())
        except OSError:
            saved = False
        session.record.update(outcome=outcome, exit_code=code, ended_unix=self.wall(),
                              map_saved=saved)
        try:
            storage.write_json(session.directory / "session.json", session.record)
        except OSError as error:
            LOG.error("cannot record session %s: %s", session.id, error)
        storage.adopt_ownership(session.directory, self.map_root)
        self.session_marker.unlink(missing_ok=True)
        self.last_session = {"id": session.id, "outcome": outcome, "map_saved": saved,
                             "directory": str(session.directory)}
        LOG.info("session %s finished: %s (exit %s, map %s)", session.id, outcome, code,
                 "saved" if saved else "not saved")


def load_provenance(path):
    """Upstream pins + wrapper identity recorded at image build (for session.json)."""
    try:
        return {"upstream": json.loads(Path(path).read_text()).get("sources", {})}
    except (OSError, ValueError):
        return {}


def service_identity():
    ref, rev = os.environ.get("GLIM_SERVICE_REF"), os.environ.get("GLIM_SERVICE_REV")
    return {"service_build": {"ref": ref or "unknown", "rev": rev or "unknown"}}
