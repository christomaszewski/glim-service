"""The supervisor's state machine, process ownership and session records — no ROS, no GLIM."""
import copy
import json
import os
from pathlib import Path
import shutil
import signal
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "supervisor"))

from glim_supervisor import controller as ctl  # noqa: E402
from glim_supervisor import storage  # noqa: E402
from glim_supervisor.process import GlimProcess  # noqa: E402

INSTANCE = {
    "version": 1, "service": "glim", "instance": "slam", "namespace": "/slam",
    "use_sim_time": False, "pipeline": "lightweight",
    "inputs": {"points": "/top/points", "imu": "/ins_main/imu/data"},
    "glim": {"glim_ros": {"points_topic": "/top/points"}},
    "map": {"save_timeout_s": 5.0, "min_free_gb": 1.0, "keep_sessions": 0},
    "resources": {"memory_guard_pct": 90.0},
    "health": {"interval_s": 1.0, "stale_after_s": 5.0, "stall_timeout_s": 5.0, "limits": {}},
}


class FakeProcess:
    """Stands in for GlimProcess; tests flip `alive` / `exit_code` to simulate GLIM."""
    instances = []

    def __init__(self, command, clock):
        self.command = command
        self.clock = clock
        self.alive = False
        self.exit_code = None
        self.started_at = None
        self.stop_outcome = ("saved", 0)
        self.dump = Path(command[command.index("-p", 6) + 1].split(":=", 1)[1])
        FakeProcess.instances.append(self)

    def start(self):
        self.alive, self.started_at = True, self.clock()

    def running(self):
        return self.alive

    def returncode(self):
        return None if self.alive else self.exit_code

    def uptime(self):
        return 0.0 if self.started_at is None else self.clock() - self.started_at

    def crash(self, code):
        self.alive, self.exit_code = False, code

    def stop(self, save_timeout_s):
        if self.alive and self.stop_outcome[0] == "saved":
            self.dump.mkdir(parents=True, exist_ok=True)
            (self.dump / "graph.txt").write_text("map")
        self.alive, self.exit_code = False, self.stop_outcome[1]
        return self.stop_outcome


class Clock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


def fake_compose(defaults_dir, instance, session_dir):
    config_dir = Path(session_dir) / "config"
    config_dir.mkdir(parents=True)
    (config_dir / "config.json").write_text("{}")
    return config_dir, []


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.map_root = self.root / "data"
        self.map_root.mkdir()
        self.clock = Clock()
        self.wall_now = 1792000000.0
        self.memory = {"used_bytes": 1 << 30, "limit_bytes": 8 << 30}
        self.free_gb = 100.0
        FakeProcess.instances = []
        self.compose = ctl.glimconfig.compose
        ctl.glimconfig.compose = fake_compose
        self.addCleanup(setattr, ctl.glimconfig, "compose", self.compose)

    def make(self, initial="active", instance=None, **kwargs):
        kwargs.setdefault("memory_probe", lambda: self.memory)
        kwargs.setdefault("disk_probe", lambda path: (self.free_gb, 50.0))
        controller = ctl.Controller(
            instance or INSTANCE, defaults_dir=self.root, map_root=self.map_root,
            target_file=self.root / "state/target", initial=initial,
            process_factory=lambda command: FakeProcess(command, self.clock),
            clock=self.clock, wall=lambda: self.wall_now, startup_timeout_s=10, **kwargs)
        # blocking waits use stop_event.wait; advance the fake clock instead of sleeping
        controller._pause = lambda seconds: self.clock.advance(seconds)
        return controller

    def session_record(self, process):
        return json.loads((process.dump.parent / "session.json").read_text())

    def test_activate_starts_a_session_and_waits_until_ready(self):
        controller = self.make(initial="standby")
        self.assertEqual(controller.state()["state"], "standby")
        controller.transition("active")
        process, = FakeProcess.instances
        self.assertEqual(controller.state()["state"], "active")
        self.assertIn("__ns:=/slam", process.command)
        self.assertIn("use_sim_time:=false", process.command)
        session_dir = process.dump.parent
        self.assertEqual(session_dir.parent, self.map_root / "maps" / "slam")
        self.assertEqual(session_dir.name, "20261014T174640Z")
        record = self.session_record(process)
        self.assertEqual((record["outcome"], record["pipeline"]), ("running", "lightweight"))
        self.assertEqual((self.root / "state/target").read_text().strip(), "active")

    def test_standby_saves_the_map_and_records_the_outcome(self):
        controller = self.make()
        controller.transition("active")
        process, = FakeProcess.instances
        controller.transition("standby")
        state = controller.state()
        self.assertEqual(state["state"], "standby")
        self.assertIn("saved", state["detail"])
        record = self.session_record(process)
        self.assertEqual((record["outcome"], record["map_saved"], record["exit_code"]), ("saved", True, 0))

    def test_each_activation_is_a_new_session(self):
        controller = self.make()
        controller.transition("active")
        controller.transition("standby")
        controller.transition("active")
        first, second = FakeProcess.instances
        self.assertNotEqual(first.dump.parent, second.dump.parent)
        self.assertEqual(second.dump.parent.name, "20261014T174640Z_2")

    def test_run_pointer_is_pinned_at_session_start(self):
        run = self.map_root / "runs" / "20261022_field"
        run.mkdir(parents=True)
        (self.map_root / "current").symlink_to(Path("runs") / run.name)
        controller = self.make()
        controller.transition("active")
        self.assertEqual(FakeProcess.instances[0].dump.parent.parent, run.resolve() / "maps" / "slam")

    def test_pointer_outside_the_mount_falls_back_to_flat_layout(self):
        outside = self.root / "elsewhere"
        outside.mkdir()
        (self.map_root / "current").symlink_to(outside)
        self.assertEqual(storage.instance_root(self.map_root, "slam"), self.map_root / "maps" / "slam")

    def test_unexpected_exit_restarts_with_backoff_then_turns_unhealthy(self):
        controller = self.make()
        controller.transition("active")
        for attempt in range(1, 4):
            FakeProcess.instances[-1].crash(-11)
            controller.tick()
            state = controller.state()
            self.assertEqual(state["state"], "transitioning")
            self.assertIn("restarting in", state["detail"])
            self.assertEqual(len(FakeProcess.instances), attempt)   # no restart before backoff
            self.clock.advance(2 ** attempt + 0.1)
            controller.tick()
            self.assertEqual(len(FakeProcess.instances), attempt + 1)
        self.assertEqual(controller.restarts, 3)
        self.assertFalse(controller.health()[0])
        crashed = self.session_record(FakeProcess.instances[0])
        self.assertEqual((crashed["outcome"], crashed["exit_code"]), ("exited", -11))
        # a session that stays up clears the failure streak
        self.clock.advance(61)
        controller.tick()
        self.assertTrue(controller.health()[0])
        self.assertEqual(controller.state()["state"], "active")

    def test_operator_activate_skips_the_backoff(self):
        controller = self.make()
        controller.transition("active")
        FakeProcess.instances[-1].crash(1)
        controller.tick()
        controller.transition("active")
        self.assertEqual(len(FakeProcess.instances), 2)
        self.assertEqual(controller.state()["state"], "active")

    def test_exit_during_startup_fails_the_transition_but_keeps_the_target(self):
        controller = self.make(initial="standby")
        original = FakeProcess.start

        def start_and_die(process):
            original(process)
            process.crash(134)
        FakeProcess.start = start_and_die
        self.addCleanup(setattr, FakeProcess, "start", original)
        with self.assertRaisesRegex(RuntimeError, "exited during startup"):
            controller.transition("active")
        self.assertEqual(controller.target, "active")
        self.assertEqual(controller.failures, 1)

    def test_compose_failure_records_the_session(self):
        controller = self.make(initial="standby")

        def broken(*_):
            raise ValueError("no `glim_ros` section")
        ctl.glimconfig.compose = broken
        with self.assertRaisesRegex(RuntimeError, "failed to start"):
            controller.transition("active")
        session_dir, = (self.map_root / "maps/slam").iterdir()
        record = json.loads((session_dir / "session.json").read_text())
        self.assertEqual(record["outcome"], "failed_to_start")

    def test_ready_probe_gates_active(self):
        controller = self.make(initial="standby")
        controller.ready_probe = lambda: False
        with self.assertRaisesRegex(RuntimeError, "not visible on the ROS graph"):
            controller.transition("active")
        self.assertEqual(controller.state()["state"], "transitioning")
        controller.ready_probe = lambda: None   # graph unknown -> trust the process
        self.assertEqual(controller.state()["state"], "active")

    def test_target_survives_a_supervisor_restart(self):
        controller = self.make()
        controller.transition("standby")
        self.assertEqual(self.make(initial="active").target, "standby")

    def test_failed_standby_save_fails_the_transition(self):
        controller = self.make()
        controller.transition("active")
        FakeProcess.instances[-1].stop_outcome = ("save_timeout", -9)
        with self.assertRaisesRegex(RuntimeError, "not saved cleanly"):
            controller.transition("standby")
        self.assertEqual(controller.state()["state"], "standby")   # GLIM is stopped regardless
        self.assertIn("save_timeout", controller.snapshot()["save_error"])

    def test_stop_after_a_failed_in_flight_save_is_not_clean(self):
        controller = self.make()
        controller.transition("active")
        process = FakeProcess.instances[-1]
        process.stop_outcome = ("failed", 1)
        original = process.stop

        def stop_during_shutdown(budget):
            controller.stop_event.set()        # docker stop arrives while standby is saving
            return original(budget)
        process.stop = stop_during_shutdown
        with self.assertRaises(RuntimeError):
            controller.transition("standby")
        self.assertFalse(controller.shutdown())

    def test_unwritable_map_root_is_a_counted_failure(self):
        controller = self.make(initial="standby")
        original = storage.new_session_dir

        def denied(*_):
            raise PermissionError(13, "Permission denied", str(self.map_root))
        ctl.storage.new_session_dir = denied
        self.addCleanup(setattr, ctl.storage, "new_session_dir", original)
        with self.assertRaisesRegex(RuntimeError, "failed to start: .*Permission denied"):
            controller.transition("active")
        for _ in range(2):
            self.clock.advance(100)
            controller.tick()
        self.assertEqual(controller.failures, 3)
        self.assertFalse(controller.health()[0])
        self.assertIn("Permission denied", controller.state()["detail"])

    def test_lifecycle_style_activate_reverts_on_failure(self):
        controller = self.make(initial="standby")
        original = FakeProcess.start

        def start_and_die(process):
            original(process)
            process.crash(134)
        FakeProcess.start = start_and_die
        self.addCleanup(setattr, FakeProcess, "start", original)
        with self.assertRaises(RuntimeError):
            controller.transition("active", revert_on_failure=True)
        self.assertEqual(controller.target, "standby")
        self.assertEqual((self.root / "state/target").read_text().strip(), "standby")
        self.clock.advance(100)
        controller.tick()
        self.assertEqual(len(FakeProcess.instances), 1)   # nothing restarts it

    def test_activate_waits_for_a_session_tick_already_started(self):
        controller = self.make()
        controller.tick()                      # restart begun by the supervise loop
        controller.ready_probe = lambda: False  # never shows up on the graph
        with self.assertRaisesRegex(RuntimeError, "not visible on the ROS graph"):
            controller.transition("active")
        controller.ready_probe = lambda: True
        controller.transition("active")         # and returns only once it is ready
        self.assertEqual(controller.state()["state"], "active")

    def test_crash_just_before_standby_is_a_lost_map(self):
        controller = self.make()
        controller.transition("active")
        FakeProcess.instances[-1].crash(-11)   # no tick in between
        with self.assertRaisesRegex(RuntimeError, "exited .code -11. before its map was saved"):
            controller.transition("standby")
        self.assertIn("before its map was saved", controller.snapshot()["save_error"])
        self.assertEqual(controller.state()["state"], "standby")

    def test_unreadable_saved_target_falls_back_and_is_rewritten(self):
        (self.root / "state").mkdir()
        for garbage in ("", "parked\n", "\x00\x00"):
            with self.subTest(garbage=garbage):
                (self.root / "state/target").write_text(garbage)
                controller = self.make(initial="standby")
                self.assertEqual(controller.target, "standby")
                self.assertEqual((self.root / "state/target").read_text(), "standby\n")

    def test_starting_target_is_persisted_for_restarts(self):
        self.make(initial="standby")
        self.assertEqual(self.make(initial="active").target, "standby")

    def test_standby_still_stops_glim_when_the_target_cannot_be_saved(self):
        controller = self.make()
        controller.transition("active")
        original = ctl.storage.write_durable

        def full_disk(*_):
            raise OSError(28, "No space left on device")
        ctl.storage.write_durable = full_disk
        self.addCleanup(setattr, ctl.storage, "write_durable", original)
        with self.assertRaisesRegex(RuntimeError, "could not be saved for restarts"):
            controller.transition("standby")
        self.assertEqual(controller.state()["state"], "standby")
        self.assertEqual(self.session_record(FakeProcess.instances[-1])["outcome"], "saved")

    def test_shutdown_stays_transitioning_while_it_saves(self):
        controller = self.make()
        controller.transition("active")
        process = FakeProcess.instances[-1]
        seen = []
        original = process.stop

        def observe(budget):
            controller.changing.clear()           # an in-flight transition's finally
            seen.append(controller.state())
            return original(budget)
        process.stop = observe
        self.assertTrue(controller.shutdown())
        self.assertEqual(seen[0]["state"], "transitioning")
        self.assertIn("shutting down: saving map", seen[0]["detail"])

    def test_interrupted_session_is_closed_out_on_restart(self):
        controller = self.make()
        controller.transition("active")
        session_dir = FakeProcess.instances[-1].dump.parent   # supervisor "dies" here
        self.assertEqual(self.session_record(FakeProcess.instances[-1])["outcome"], "running")
        restarted = self.make()
        restarted.recover_interrupted()
        record = json.loads((session_dir / "session.json").read_text())
        self.assertEqual((record["outcome"], record["map_saved"]), ("interrupted", False))
        self.assertFalse(restarted.session_marker.exists())
        restarted.recover_interrupted()   # idempotent

    def test_a_dump_cut_short_is_not_a_saved_map(self):
        controller = self.make()
        controller.transition("active")
        process = FakeProcess.instances[-1]
        process.dump.mkdir(parents=True)
        (process.dump / "000000").write_text("partial")
        process.stop_outcome = ("save_timeout", -9)
        with self.assertRaises(RuntimeError):
            controller.transition("standby")
        self.assertFalse(self.session_record(process)["map_saved"])

    def test_parking_clears_a_crash_loop_and_standby_is_healthy(self):
        controller = self.make()
        controller.transition("active")
        for _ in range(3):
            FakeProcess.instances[-1].crash(1)
            controller.tick()
            self.clock.advance(100)
            controller.tick()
        self.assertFalse(controller.health()[0])
        controller.transition("standby")
        self.assertTrue(controller.health()[0])
        self.assertEqual(controller.failures, 0)

    def test_memory_guard_saves_and_continues_with_a_fresh_map(self):
        controller = self.make()
        controller.transition("active")
        first = FakeProcess.instances[-1]
        self.memory = {"used_bytes": int(7.3 * (1 << 30)), "limit_bytes": 8 << 30}   # 91%
        controller.tick()
        record = self.session_record(first)
        self.assertEqual((record["outcome"], record["ended_by"], record["map_saved"]),
                         ("saved", "memory guard", True))
        self.memory = {"used_bytes": 1 << 30, "limit_bytes": 8 << 30}
        controller.tick()
        self.assertEqual(len(FakeProcess.instances), 2)            # mapping continues
        self.assertIn("memory guard", controller.snapshot()["guard_event"])
        self.assertTrue(controller.health()[0])                     # not a failure

    def test_disk_guard_refuses_to_start_and_ends_a_running_session(self):
        controller = self.make(initial="standby")
        self.free_gb = 0.4
        with self.assertRaisesRegex(RuntimeError, "only 0.40 GB free"):
            controller.transition("active")
        self.assertFalse((self.map_root / "maps").exists())       # nothing written to a full disk
        self.free_gb = 50.0
        controller.transition("active")
        self.free_gb = 0.5
        controller.tick()
        self.assertEqual(self.session_record(FakeProcess.instances[-1])["ended_by"], "disk guard")
        controller.tick()
        self.assertEqual(len(FakeProcess.instances), 1)            # held until space returns
        self.free_gb = 50.0
        self.clock.advance(100)
        controller.tick()
        self.assertEqual(len(FakeProcess.instances), 2)

    def test_retention_applies_to_the_fallback_volume_only(self):
        instance = dict(INSTANCE, map=dict(INSTANCE["map"], keep_sessions=2))
        for store, expected in (("datadir", 4), ("volume", 2)):
            with self.subTest(store=store):
                shutil.rmtree(self.map_root / "maps", ignore_errors=True)
                controller = self.make(instance=instance, map_store=store)
                for i in range(4):
                    self.wall_now = 1792000000.0 + 60 * i
                    controller.transition("active")
                    controller.transition("standby")
                self.assertEqual(len(list((self.map_root / "maps/slam").iterdir())), expected)

    def test_heartbeat_and_orphan_closeout(self):
        controller = self.make()
        controller.transition("active")
        self.clock.advance(31)
        self.wall_now += 31
        controller.tick()
        session_dir = FakeProcess.instances[-1].dump.parent
        self.assertEqual(self.session_record(FakeProcess.instances[-1])["heartbeat_unix"], self.wall_now)
        # its supervisor dies with the container (no marker survives); a new one starts later
        (self.root / "state").joinpath("current-session").unlink()
        self.wall_now += 60
        fresh = self.make()
        fresh.recover_interrupted()
        self.assertEqual(json.loads((session_dir / "session.json").read_text())["outcome"], "running")
        self.wall_now += 120                                         # heartbeat now stale
        fresh.recover_interrupted()
        self.assertEqual(json.loads((session_dir / "session.json").read_text())["outcome"],
                         "interrupted")

    def test_input_warnings_are_per_session(self):
        controller = self.make()
        controller.transition("active")
        FakeProcess.instances[-1].on_line("large time difference between points and imu!!")
        self.assertIn("time_sync", controller.snapshot()["input_warnings"])
        controller.transition("standby")
        controller.transition("active")
        self.assertEqual(controller.snapshot()["input_warnings"], {})

    def test_extrinsic_is_read_from_tf_at_session_start(self):
        instance = copy.deepcopy(INSTANCE)
        instance["extrinsics"] = {"source": "tf", "lidar_frame": "ouster_lidar", "imu_frame": "imu_link"}
        controller = self.make(initial="standby", instance=instance)
        with self.assertRaisesRegex(RuntimeError, "needs the ROS interface"):
            controller.transition("active")
        asked = []
        controller.extrinsic_resolver = lambda lidar, imu: asked.append((lidar, imu)) or [0, 0, -0.1, 0, 0, 0, 1]
        controller.transition("active")
        record = self.session_record(FakeProcess.instances[-1])
        self.assertEqual(asked, [("ouster_lidar", "imu_link")])
        self.assertEqual(record["extrinsics"]["source"], "tf")
        self.assertEqual(record["extrinsics"]["T_lidar_imu"], [0, 0, -0.1, 0, 0, 0, 1])

    def test_finished_sessions_are_scored_against_their_reference(self):
        instance = copy.deepcopy(INSTANCE)
        instance["evaluation"] = {"reference_topic": "/ins_main/odom", "max_dt_s": 0.05}
        controller = self.make(instance=instance)
        closed = []
        controller.session_end_hooks.append(lambda session: closed.append(session.id))
        controller.transition("active")
        process = FakeProcess.instances[-1]
        session_dir = process.dump.parent
        poses = "".join(f"{100 + i * 0.1:.3f} {i * 0.5:.3f} 0 0 0 0 0 1\n" for i in range(50))
        (session_dir / "reference.tum").write_text(poses)
        original = process.stop

        def stop_and_write_trajectory(budget):
            result = original(budget)
            (process.dump / "traj_imu.txt").write_text(poses)
            return result
        process.stop = stop_and_write_trajectory
        controller.transition("standby")
        record = self.session_record(process)
        self.assertEqual(closed, [session_dir.name])
        self.assertLess(record["evaluation"]["ate_rmse_m"], 1e-6)
        self.assertTrue((session_dir / "evaluation.json").is_file())

    def test_remap_runs_glim_rosbag_into_a_new_session(self):
        class Rosbag(FakeProcess):
            def start(self):
                super().start()
                self.dump.mkdir(parents=True)
                (self.dump / "graph.bin").write_text("map")
                self.on_line("[glim] [warning] large time gap between consecutive LiDAR frames!!")
                self.crash(0)   # glim_rosbag exits by itself when the bags end (auto_quit)
        controller = ctl.Controller(
            INSTANCE, defaults_dir=self.root, map_root=self.map_root,
            target_file=self.root / "state/target", process_factory=lambda c: Rosbag(c, self.clock),
            clock=self.clock, wall=lambda: self.wall_now, memory_probe=lambda: None,
            disk_probe=lambda path: (100.0, 50.0))
        references = []
        record = controller.remap([Path("/remap/input/bag_0")], "glim_rosbag",
                                  reference=references.append)
        command = Rosbag.instances[-1].command
        self.assertEqual(command[:3], ["glim_rosbag", "/remap/input/bag_0", "--ros-args"])
        self.assertIn("auto_quit:=true", command)
        self.assertEqual((record["mode"], record["outcome"], record["map_saved"]), ("remap", "saved", True))
        self.assertEqual(record["input_warnings"], {"lidar_gap": 1})
        self.assertTrue(record["session"].endswith("-remap"))
        self.assertEqual(references, [self.map_root / "maps/slam" / record["session"]])

    def test_shutdown_saves_and_reports_failures(self):
        controller = self.make()
        controller.transition("active")
        self.assertTrue(controller.shutdown())
        controller = self.make()
        controller.tick()
        FakeProcess.instances[-1].stop_outcome = ("save_timeout", -9)
        self.assertFalse(controller.shutdown())
        self.assertEqual(controller.state()["state"], "transitioning")

    def test_transitions_do_not_queue(self):
        controller = self.make()

        class Busy:  # another transition holds the lock past the bounded wait
            def acquire(self, blocking=True, timeout=-1):
                return False
        controller.lock = Busy()
        with self.assertRaisesRegex(RuntimeError, "transition in progress"):
            controller.transition("standby")
        controller.tick()   # the supervise loop just skips its turn
        self.assertEqual(controller.target, "active")

    def test_state_vocabulary(self):
        controller = self.make()
        seen = {controller.state()["state"]}
        controller.transition("active")
        seen.add(controller.state()["state"])
        controller.transition("standby")
        seen.add(controller.state()["state"])
        self.assertLessEqual(seen, {"active", "standby", "transitioning"})


class GlimProcessTests(unittest.TestCase):
    """The real process wrapper against stand-in programs."""

    def program(self, body):
        return [sys.executable, "-c", "import signal, sys, time\n" + body]

    def test_sigint_lets_the_process_save(self):
        process = GlimProcess(self.program(
            "signal.signal(signal.SIGINT, lambda *_: sys.exit(0))\n"
            "print('ready', flush=True)\ntime.sleep(60)"))
        process.start()
        time.sleep(0.5)
        self.assertTrue(process.running())
        self.assertEqual(process.stop(5), ("saved", 0))

    def test_escalates_past_the_save_budget(self):
        process = GlimProcess(self.program(
            "signal.signal(signal.SIGINT, signal.SIG_IGN)\ntime.sleep(60)"))
        process.start()
        time.sleep(0.5)
        outcome, code = process.stop(0.5)
        self.assertEqual(outcome, "save_timeout")
        self.assertEqual(code, -signal.SIGTERM)

    def test_already_exited(self):
        process = GlimProcess([sys.executable, "-c", "raise SystemExit(3)"])
        process.start()
        process.popen.wait()
        self.assertEqual(process.stop(1), ("exited", 3))

    def test_sigint_before_rclcpp_installs_its_handler_is_not_a_failed_save(self):
        # A C++ process before rclcpp::init dies by SIGINT itself (Python would raise
        # KeyboardInterrupt instead), so restore the default action to emulate it exactly.
        process = GlimProcess([sys.executable, "-c",
                               "import signal, time\nsignal.signal(signal.SIGINT, signal.SIG_DFL)\n"
                               "time.sleep(60)"])
        process.start()
        time.sleep(0.3)
        self.assertEqual(process.stop(5), ("not_started", -signal.SIGINT))
        self.assertTrue(ctl.stopped_cleanly("not_started", -signal.SIGINT))

    def test_output_is_forwarded_and_observed(self):
        seen = []
        process = GlimProcess([sys.executable, "-c",
                               "import sys; print('[glim] [warning] IMU timestamp rewind detected!!');"
                               "sys.stdout.flush(); print('done', file=sys.stderr)"],
                              on_line=seen.append)
        process.start()
        process.popen.wait()
        process.reader.join(timeout=5)
        self.assertEqual([line.strip() for line in seen],
                         ["[glim] [warning] IMU timestamp rewind detected!!", "done"])

    def test_nonzero_exit_on_save_is_a_failure(self):
        process = GlimProcess(self.program(
            "signal.signal(signal.SIGINT, lambda *_: sys.exit(4))\ntime.sleep(60)"))
        process.start()
        time.sleep(0.5)
        self.assertEqual(process.stop(5), ("failed", 4))


class OwnershipTests(unittest.TestCase):
    @unittest.skipUnless(hasattr(os, "geteuid") and os.geteuid() == 0, "needs root to chown")
    def test_session_files_follow_the_data_dir_owner(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            os.chown(root, 4242, 4242)
            (root / "s" / "dump").mkdir(parents=True)
            (root / "s" / "dump" / "graph.txt").write_text("x")
            storage.adopt_ownership(root / "s", root)
            self.assertEqual((root / "s/dump/graph.txt").stat().st_uid, 4242)


if __name__ == "__main__":
    unittest.main()
