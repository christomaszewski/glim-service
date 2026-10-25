"""Operational state for one GLIM instance: the persisted target, mapping sessions, recovery.

active  = target active and a GLIM session is running and visible on the ROS graph
standby = target standby and no session (the last one's map is saved)
anything else is `transitioning` (starting, saving a map, waiting to restart after an exit).

Only the controller starts or stops GLIM. Operator transitions block until the new state is
reached or fail with a reason; the supervise loop's tick() converges everything else (a GLIM
exit while active is restarted with capped exponential backoff, each restart a new session).
"""
import copy
import json
import logging
import os
from pathlib import Path
import shutil
import threading
import time

from . import glimconfig, glimlog, resources, storage, trajectory
from .process import GlimProcess

LOG = logging.getLogger("glim-supervisor")

ACTIVE, STANDBY = "active", "standby"
READY_AFTER_S = 2.0       # GLIM must stay up this long before a session counts as started
UNHEALTHY_FAILURES = 3    # consecutive failed sessions before the container reports unhealthy
HEARTBEAT_S = 30.0        # a running session's record is refreshed this often...
ORPHAN_AFTER_S = 120.0    # ...so one not refreshed for this long belongs to a dead supervisor
GUARD_EVENT_S = 120.0     # how long a guard intervention stays visible on /diagnostics


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
                 startup_timeout_s=60.0, stable_after_s=60.0, max_backoff_s=60.0,
                 memory_probe=resources.memory, disk_probe=storage.disk_free, map_store="datadir"):
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
        self.min_free_gb = float(instance["map"].get("min_free_gb", 0.0))
        self.keep_sessions = int(instance["map"].get("keep_sessions", 0))
        self.memory_guard_pct = float((instance.get("resources") or {}).get("memory_guard_pct", 90.0))
        self.memory_probe = memory_probe
        self.disk_probe = disk_probe
        self.map_store = map_store   # "volume" (the fallback store) or "datadir"
        self.ready_probe = None   # optional callable -> bool | None (the ROS graph check)
        # optional callable (lidar_frame, imu_frame) -> [x y z qx qy qz qw] from TF (ROS side)
        self.extrinsic_resolver = None
        self.session_end_hooks = []   # called with the Session before it is finalized
        self.input_warnings = glimlog.InputWarnings(clock)
        self.guard_event = None   # (message, monotonic time) of the last guard intervention
        self.last_heartbeat = 0.0
        self.last_orphan_scan = 0.0

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
            "input_warnings": self.input_warnings.snapshot() if session else {},
            "memory": self._memory(),
            "memory_guard_pct": self.memory_guard_pct,
            "guard_event": (self.guard_event[0] if self.guard_event and
                            self.clock() - self.guard_event[1] < GUARD_EVENT_S else ""),
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
                self._guards()
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

    # --- guards (run from tick, under the lock) --------------------------------------------
    def _memory(self):
        try:
            reading = self.memory_probe()
        except Exception:
            return None
        if reading and reading.get("limit_bytes"):
            reading = dict(reading, used_pct=100.0 * reading["used_bytes"] / reading["limit_bytes"])
        return reading

    def _free_gb(self):
        root = storage.instance_root(self.map_root, self.instance["instance"])
        probe = next((p for p in (root, *root.parents) if p.exists()), self.map_root)
        try:
            return self.disk_probe(probe)[0]
        except OSError:
            return None

    def _guards(self):
        """Heartbeat the running session, and end it cleanly — map saved — before the kernel's
        OOM killer or a full disk would end it uncleanly."""
        now = self.clock()
        if now - self.last_orphan_scan >= 300:
            self.last_orphan_scan = now
            self._close_orphans()
        session = self.session
        if session is None or not session.process.running():
            return
        if now - self.last_heartbeat >= HEARTBEAT_S:
            self.last_heartbeat = now
            session.record["heartbeat_unix"] = self.wall()
            try:
                storage.write_json(session.directory / "session.json", session.record)
            except OSError:
                pass
        reason = None
        memory = self._memory()
        if memory and memory.get("used_pct", 0) >= self.memory_guard_pct:
            reason = (f"memory guard: GLIM uses {memory['used_pct']:.0f}% of the container's "
                      f"{memory['limit_bytes'] / 1e9:.1f} GB limit")
        if reason is None and self.min_free_gb > 0:
            free = self._free_gb()
            if free is not None and free < self.min_free_gb:
                reason = (f"disk guard: {free:.2f} GB free < map.min_free_gb "
                          f"{self.min_free_gb:g}")
        if reason is None:
            return
        LOG.warning("%s; ending session %s now (saving its map) rather than lose it", reason,
                    session.id)
        session.record["ended_by"] = reason.split(":", 1)[0]
        self.guard_event = (f"{reason}; ended session {session.id}", now)
        self._stop_session()
        # Memory: continue at once with a fresh map. Disk: the start check holds new sessions back
        # until space returns (and reports why).
        self.next_start_at = now

    def _close_orphans(self):
        """Mark sessions whose supervisor died without finishing them (no fresh heartbeat)."""
        root = storage.instance_root(self.map_root, self.instance["instance"])
        current = self.session.directory if self.session else None
        try:
            candidates = [d for d in root.iterdir() if d.is_dir() and d != current]
        except OSError:
            return
        for directory in candidates:
            try:
                record = json.loads((directory / "session.json").read_text())
            except (OSError, ValueError):
                continue
            beat = record.get("heartbeat_unix") or record.get("started_unix") or 0
            if record.get("outcome") == "running" and self.wall() - beat > ORPHAN_AFTER_S:
                record.update(outcome="interrupted", map_saved=False, ended_unix=None)
                try:
                    storage.write_json(directory / "session.json", record)
                    LOG.warning("session %s was orphaned (no heartbeat since %.0fs); marked "
                                "interrupted", directory.name, self.wall() - beat)
                except OSError:
                    pass

    def _prune(self):
        """keep_sessions for the fallback volume only: a vehicle data dir's runs are rig's to
        manage (`rig run rm`), and a session inside a run is part of its record."""
        if self.map_store != "volume" or self.keep_sessions <= 0:
            return
        root = storage.instance_root(self.map_root, self.instance["instance"])
        current = self.session.directory if self.session else None
        sessions = sorted(d for d in root.iterdir() if d.is_dir() and d != current)
        for directory in sessions[:max(0, len(sessions) - self.keep_sessions)]:
            try:
                if json.loads((directory / "session.json").read_text()).get("outcome") == "running":
                    continue
            except (OSError, ValueError):
                pass
            LOG.info("map.keep_sessions=%d: removing oldest session %s", self.keep_sessions,
                     directory.name)
            shutil.rmtree(directory, ignore_errors=True)

    def _prepare_session(self, mode, **extra):
        """Allocate a session directory, compose its GLIM config, write its first record.
        -> (directory, config_dir, instance used, record). Raises with the reason."""
        stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime(self.wall()))
        record = {
            "version": 1,
            "instance": self.instance["instance"],
            "mode": mode,
            "pipeline": self.instance.get("pipeline"),
            "inputs": self.instance.get("inputs"),
            "use_sim_time": bool(self.instance.get("use_sim_time")),
            "started_unix": self.wall(),
            "outcome": "running",
            **extra,
            **self.provenance,
        }
        if self.min_free_gb > 0:
            free = self._free_gb()
            if free is not None and free < self.min_free_gb:
                raise RuntimeError(f"only {free:.2f} GB free for maps (map.min_free_gb "
                                   f"{self.min_free_gb:g}); free space to start mapping")
        instance = self._session_instance(record)
        root = storage.instance_root(self.map_root, self.instance["instance"])
        created = [p for p in (root.parent, root) if not p.exists()]
        directory = storage.new_session_dir(root, stamp + ("-remap" if mode == "remap" else ""))
        for path in created:
            storage.adopt_ownership(path, self.map_root, recursive=False)
        record["session"] = directory.name
        try:
            config_dir, warnings = glimconfig.compose(self.defaults_dir, instance, directory)
            self.config_warnings = warnings
            for warning in warnings:
                LOG.warning("%s", warning)
            record["config_warnings"] = warnings
            storage.write_json(directory / "session.json", record)
        except Exception as error:
            record.update(outcome="failed_to_start", error=str(error), ended_unix=self.wall())
            try:
                storage.write_json(directory / "session.json", record)
                storage.adopt_ownership(directory, self.map_root)
            except OSError:
                pass
            raise RuntimeError(f"session {directory.name}: {error}") from error
        return directory, config_dir, instance, record

    def _session_instance(self, record):
        """The instance this session runs with: the extrinsic from TF, when configured so."""
        extrinsics = self.instance.get("extrinsics") or {}
        if extrinsics.get("source") != "tf":
            record["extrinsics"] = {"source": "config", "T_lidar_imu":
                                    (self.instance["glim"].get("sensors") or {}).get("T_lidar_imu")}
            return self.instance
        if self.extrinsic_resolver is None:
            raise RuntimeError("extrinsics.from_tf needs the ROS interface (TF), which is not up")
        value = self.extrinsic_resolver(extrinsics["lidar_frame"], extrinsics["imu_frame"])
        instance = copy.deepcopy(self.instance)
        instance["glim"].setdefault("sensors", {})["T_lidar_imu"] = value
        record["extrinsics"] = {"source": "tf", "T_lidar_imu": value,
                                "lidar_frame": extrinsics["lidar_frame"],
                                "imu_frame": extrinsics["imu_frame"]}
        LOG.info("T_lidar_imu from TF %s <- %s: %s", extrinsics["lidar_frame"],
                 extrinsics["imu_frame"], value)
        return instance

    def _start_session(self, wait):
        # Everything up to a running process counts as starting: an unwritable or full data dir,
        # or a missing TF extrinsic, must become a visible, backed-off failure.
        try:
            directory, config_dir, instance, record = self._prepare_session("live")
            self.activity = f"starting session {directory.name}"
            process = self.process_factory(glim_command(self.glim_binary, instance, config_dir,
                                                        directory / "dump"))
            self.input_warnings.reset()
            process.on_line = self.input_warnings.observe
            process.start()
        except Exception as error:
            self._note_failure(f"a new session failed to start: {error}")
            raise RuntimeError(self.error) from error
        session = Session(directory.name, directory, process, record)
        self.session = session
        try:   # lets a restarted supervisor close out a session it never got to finish
            storage.write_durable(self.session_marker, str(directory) + "\n")
        except OSError as error:
            LOG.warning("cannot record the running session: %s", error)
        if wait:
            self._await_ready(session)

    def remap(self, bags, binary, reference=None):
        """Map recorded bags offline, as fast as GLIM can (glim_rosbag), into a new session.
        `reference(session_dir)` may write the session's reference.tum (from the same bags).
        -> the session record. Blocking; runs outside the supervise loop (its own container)."""
        directory, config_dir, instance, record = self._prepare_session(
            "remap", source_bags=[str(b) for b in bags])
        if reference is not None:
            try:
                reference(directory)
            except Exception as error:   # evaluation is optional; the map is the point
                LOG.warning("no evaluation reference from the bags: %s", error)
        command = [binary, *map(str, bags), *glim_command("", instance, config_dir,
                                                          directory / "dump")[1:],
                   "-p", "auto_quit:=true"]
        self.input_warnings.reset()
        process = self.process_factory(command)
        process.on_line = self.input_warnings.observe
        session = Session(directory.name, directory, process, record)
        process.start()
        LOG.info("remapping %s into session %s", ", ".join(map(str, bags)), directory.name)
        code = process.popen.wait() if getattr(process, "popen", None) else process.returncode()
        if getattr(process, "reader", None) is not None:
            process.reader.join(timeout=5)
        record["input_warnings"] = {c: w["count"] for c, w in self.input_warnings.snapshot().items()}
        self._finish(session, "saved" if code == 0 else "failed", code)
        return session.record

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
        """Close out sessions a dead supervisor never finished (SIGKILL past the grace period, OOM,
        power loss), so no record claims a session that is not running: the one this container's
        marker names, and any other whose heartbeat stopped."""
        try:
            directory = Path(self.session_marker.read_text().strip())
        except OSError:
            directory = None
        if directory is not None:
            try:
                record = json.loads((directory / "session.json").read_text())
                if record.get("outcome") == "running":
                    record.update(outcome="interrupted", map_saved=False, ended_unix=None)
                    storage.write_json(directory / "session.json", record)
                    LOG.warning("session %s was interrupted (supervisor stopped without "
                                "finishing it)", directory.name)
            except (OSError, ValueError) as error:
                LOG.warning("cannot close out interrupted session %s: %s", directory, error)
            self.session_marker.unlink(missing_ok=True)
        self._close_orphans()

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

    def _evaluate(self, session):
        spec = self.instance.get("evaluation")
        if not spec:
            return
        reference = session.directory / "reference.tum"
        estimate = next((session.directory / "dump" / name for name in ("traj_imu.txt", "odom_imu.txt")
                         if (session.directory / "dump" / name).is_file()), None)
        if estimate is None or not reference.is_file():
            session.record["evaluation"] = {"error": "no " + ("reference trajectory" if estimate
                                                              else "GLIM trajectory") + " to compare"}
            return
        try:
            result = trajectory.evaluate(estimate, reference, spec["max_dt_s"])
            storage.write_json(session.directory / "evaluation.json", result)
            session.record["evaluation"] = {key: result[key] for key in (
                "ate_rmse_m", "ate_max_m", "final_error_pct_of_path", "matched",
                "reference_path_length_m")}
            LOG.info("session %s: ATE %.3f m RMSE over %.0f m (%d poses)", session.id,
                     result["ate_rmse_m"], result["reference_path_length_m"], result["matched"])
        except (OSError, ValueError) as error:
            session.record["evaluation"] = {"error": str(error)}

    def _finish(self, session, outcome, code):
        for hook in self.session_end_hooks:
            try:
                hook(session)
            except Exception as error:
                LOG.warning("session end hook failed: %s", error)
        dump = session.directory / "dump"
        try:   # a dump cut short by SIGTERM/SIGKILL is not a saved map, however many files it has
            saved = stopped_cleanly(outcome, code) and dump.is_dir() and any(dump.iterdir())
        except OSError:
            saved = False
        session.record.update(outcome=outcome, exit_code=code, ended_unix=self.wall(),
                              map_saved=saved)
        ply = session.directory / "map.ply"
        if ply.is_file():
            session.record["map_ply_bytes"] = ply.stat().st_size
        self._evaluate(session)
        try:
            storage.write_json(session.directory / "session.json", session.record)
        except OSError as error:
            LOG.error("cannot record session %s: %s", session.id, error)
        storage.adopt_ownership(session.directory, self.map_root)
        self.session_marker.unlink(missing_ok=True)
        try:
            self._prune()
        except OSError as error:
            LOG.warning("session retention failed: %s", error)
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
