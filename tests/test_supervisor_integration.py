"""The real supervisor process (`python3 -m glim_supervisor serve`) around a fake glim_rosnode.

Covers what unit tests cannot: the serve loop, signal handling and exit codes, the control socket
end to end, session directories on disk, and — where rclpy is available (inside the image, as CI
runs it) — the lifecycle node, the ROS ready probe and /diagnostics.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "tests"))

import render_config  # noqa: E402
from test_glimconfig import upstream_defaults  # noqa: E402

try:
    import importlib.util
    HAVE_ROS = importlib.util.find_spec("rclpy") is not None
except ImportError:
    HAVE_ROS = False

FAKE_GLIM = ROOT / "tests" / "fake_glim_rosnode.py"
MINIMAL_DEFAULTS = {  # the sections GLIM's files carry, for hosts without the upstream checkout
    "config_ros.json": {"glim_ros": {"acc_scale": 0.0, "extension_modules": []}},
    "config_logging.json": {"logging": {"log_dir": "/tmp", "save_logs": True}},
    "config_viewer.json": {},
    "config_sensors.json": {"sensors": {"T_lidar_imu": [0, 0, 0, 0, 0, 0, 1], "imu_acc_noise": 0.05,
                                        "imu_gyro_noise": 0.02}},
    "config_preprocess.json": {"preprocess": {"distance_near_thresh": 0.5, "distance_far_thresh": 100.0,
                                              "random_downsample_target": 10000, "num_threads": 2}},
    "config_odometry_cpu.json": {"odometry_estimation": {"so_name": "libodometry_estimation_cpu.so"}},
    "config_sub_mapping_cpu.json": {"sub_mapping": {"so_name": "libsub_mapping.so"}},
    "config_sub_mapping_passthrough.json": {"sub_mapping": {"so_name": "libsub_mapping_passthrough.so"}},
    "config_global_mapping_cpu.json": {"global_mapping": {"so_name": "libglobal_mapping.so"}},
    "config_global_mapping_pose_graph.json": {"global_mapping": {"so_name": "libglobal_mapping_pose_graph.so"}},
}


class SupervisorProcess(unittest.TestCase):
    save_timeout_s = 5.0
    # Where rclpy exists the supervisor's ready probe requires GLIM on the ROS graph, so the fake
    # must join it too (isolated: own domain, localhost-only discovery).
    ros = HAVE_ROS

    def setUp(self):
        # AF_UNIX paths are short (104 bytes on macOS): keep the temporary directory name small.
        temp = tempfile.TemporaryDirectory(prefix="gsi")
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.map_root = self.root / "data"
        self.map_root.mkdir()
        defaults = upstream_defaults()
        if defaults is None:
            defaults = self.root / "defaults"
            defaults.mkdir()
            for name, document in MINIMAL_DEFAULTS.items():
                (defaults / name).write_text(json.dumps(document))
        cfg = yaml.safe_load((ROOT / "autonomy/glim.example.yaml").read_text())
        cfg["map"] = {"save_timeout_s": self.save_timeout_s}
        cfg["health"]["interval_s"] = 0.5
        instance = self.root / "instance.json"
        instance.write_text(json.dumps(render_config.render_instance(cfg)))
        self.control = self.root / "glim-mode"
        self.seed = self.root / "target"
        self.seed.write_text("active\n")
        fake = self.root / "glim_rosnode"
        fake.write_text(f"#!{sys.executable}\n" + FAKE_GLIM.read_text())
        fake.chmod(0o755)
        self.env = {k: v for k, v in os.environ.items() if not k.startswith(("GLIM_", "FAKE_"))}
        self.env.update(
            PYTHONPATH=os.pathsep.join(filter(None, [str(ROOT / "supervisor"),
                                                     os.environ.get("PYTHONPATH")])),
            GLIM_INSTANCE_FILE=str(instance),
            GLIM_TARGET_FILE=str(self.root / "state" / "target-state"),
            GLIM_TARGET_SEED=str(self.seed), GLIM_DEFAULTS_DIR=str(defaults),
            GLIM_UPSTREAM_SOURCES=str(self.root / "missing.json"), GLIM_ROSNODE=str(fake),
            GLIM_MAP_ROOT=str(self.map_root), GLIM_STATE_SOCKET=str(self.root / "s.sock"),
            FAKE_GLIM_CONTROL=str(self.control), VEHICLE_ID="7")
        if self.ros:
            self.env.update(FAKE_GLIM_ROS="1", RMW_IMPLEMENTATION="rmw_fastrtps_cpp",
                            ROS_DOMAIN_ID=str(150 + os.getpid() % 50),
                            ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST")
        self.process = None

    def start(self):
        self.log = open(self.root / "supervisor.log", "w")
        self.addCleanup(self.log.close)
        self.process = subprocess.Popen([sys.executable, "-m", "glim_supervisor", "serve"],
                                        env=self.env, stdout=self.log, stderr=subprocess.STDOUT)
        self.addCleanup(self.kill)
        self.wait_for(lambda: self.cli("state")[0] == 0, "control socket")

    def kill(self):
        if self.process and self.process.poll() is None:
            self.process.kill()
            self.process.wait()
        subprocess.run(["pkill", "-f", str(self.root / "glim_rosnode")], capture_output=True)

    def logs(self):
        return (self.root / "supervisor.log").read_text()

    def cli(self, command, timeout=60):
        result = subprocess.run([sys.executable, "-m", "glim_supervisor", command], env=self.env,
                                capture_output=True, text=True, timeout=timeout)
        return result.returncode, result.stdout, result.stderr

    def state(self):
        code, out, err = self.cli("state")
        self.assertEqual(code, 0, err)
        return json.loads(out)

    def wait_for(self, predicate, what, timeout=30.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                self.fail(f"supervisor exited ({self.process.returncode}) waiting for {what}:\n"
                          + self.logs())
            if predicate():
                return
            time.sleep(0.2)
        self.fail(f"timed out waiting for {what}; last state {self.cli('state')[1]}\n" + self.logs())

    def wait_state(self, state, timeout=30.0):
        self.wait_for(lambda: self.state()["state"] == state, state, timeout)

    def stop(self):
        self.process.send_signal(signal.SIGTERM)
        return self.process.wait(timeout=self.save_timeout_s + 30)

    def sessions(self):
        root = self.map_root / "maps" / "slam"
        return sorted(root.iterdir()) if root.exists() else []

    def record(self, session):
        return json.loads((session / "session.json").read_text())


class SupervisorIntegrationTests(SupervisorProcess):
    def test_session_lifecycle_and_clean_shutdown(self):
        self.start()
        self.wait_state("active")
        first, = self.sessions()
        argv = json.loads((first / "argv.json").read_text())
        self.assertIn("__ns:=/slam", argv)
        self.assertTrue((first / "config" / "config.json").is_file())
        code, _, err = self.cli("standby")
        self.assertEqual(code, 0, err)
        self.assertEqual(self.state()["state"], "standby")
        self.assertEqual(self.record(first)["outcome"], "saved")
        self.assertTrue((first / "dump" / "graph.txt").is_file())
        self.assertEqual(self.cli("activate")[0], 0)
        self.wait_state("active")
        self.assertEqual(len(self.sessions()), 2)
        self.assertEqual(self.cli("health")[0], 0)
        self.assertEqual(self.stop(), 0)
        last = self.record(self.sessions()[-1])
        self.assertEqual((last["outcome"], last["map_saved"]), ("saved", True))
        self.assertFalse((self.root / "s.sock").exists())

    def test_seeded_standby_never_opens_a_session(self):
        self.seed.write_text("standby\n")
        self.start()
        self.wait_state("standby")
        time.sleep(1.5)   # a few supervise ticks
        self.assertEqual(self.sessions(), [])
        self.assertEqual(self.stop(), 0)

    def test_persisted_target_beats_the_seed(self):
        (self.root / "state").mkdir()
        (self.root / "state" / "target-state").write_text("standby\n")
        self.start()
        self.wait_state("standby")
        self.assertEqual(self.sessions(), [])
        self.stop()

    def test_failed_save_on_stop_exits_nonzero(self):
        self.control.write_text("fail_save")
        self.start()
        self.wait_state("active")
        self.assertEqual(self.stop(), 1)
        self.assertEqual(self.record(self.sessions()[-1])["outcome"], "failed")

    def test_crash_loop_turns_unhealthy_and_an_operator_activate_recovers(self):
        self.control.write_text("crash")
        self.start()
        self.wait_for(lambda: self.cli("health")[0] == 1, "unhealthy after repeated crashes", 40)
        self.assertIn("restarting in", self.state()["detail"])
        self.control.write_text("save")
        self.assertEqual(self.cli("activate")[0], 0)   # skips the backoff
        self.wait_state("active")
        self.assertGreaterEqual(len(self.sessions()), 4)
        self.assertEqual(self.stop(), 0)

    def test_concurrent_requests_do_not_queue_or_wedge(self):
        self.control.write_text("slow_save")
        self.start()
        self.wait_state("active")
        standby = subprocess.Popen([sys.executable, "-m", "glim_supervisor", "standby"],
                                   env=self.env, stdout=subprocess.DEVNULL,
                                   stderr=subprocess.PIPE, text=True)
        self.wait_for(lambda: self.state()["state"] == "transitioning", "a transition in flight")
        self.assertEqual(self.cli("health")[0], 0)   # probes never wait on a transition
        self.assertEqual(standby.wait(timeout=30), 0, standby.stderr.read())
        self.assertEqual(self.state()["state"], "standby")
        self.assertEqual(self.stop(), 0)


class HangingSaveTests(SupervisorProcess):
    save_timeout_s = 1.0

    def test_save_budget_escalates_and_is_reported(self):
        self.control.write_text("hang")
        self.start()
        self.wait_state("active")
        code, _, err = self.cli("standby")
        self.assertEqual(code, 1)
        self.assertIn("not saved cleanly", err)
        self.assertEqual(self.state()["state"], "standby")
        self.assertEqual(self.record(self.sessions()[-1])["outcome"], "save_timeout")
        self.assertEqual(self.stop(), 0)   # nothing unsaved remains at shutdown


@unittest.skipUnless(HAVE_ROS, "rclpy not available (runs inside the image in CI)")
class RosIntegrationTests(SupervisorProcess):
    ros = True

    def setUp(self):
        super().setUp()
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        os.environ.update({k: self.env[k] for k in ("RMW_IMPLEMENTATION", "ROS_DOMAIN_ID",
                                                     "ROS_AUTOMATIC_DISCOVERY_RANGE")})
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.addCleanup(rclpy.shutdown)
        self.node = rclpy.create_node("glim_test_probe")
        self.addCleanup(self.node.destroy_node)

    def call(self, service_type, name, request, timeout=30.0):
        import rclpy
        client = self.node.create_client(service_type, name)
        self.assertTrue(client.wait_for_service(timeout_sec=timeout), f"{name} not available")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        self.assertTrue(future.done(), f"{name} did not answer")
        return future.result()

    def lifecycle_state(self):
        from lifecycle_msgs.srv import GetState
        return self.call(GetState, "/slam/glim_supervisor/get_state", GetState.Request()).current_state.label

    def change(self, transition_id):
        from lifecycle_msgs.srv import ChangeState
        request = ChangeState.Request()
        request.transition.id = transition_id
        return self.call(ChangeState, "/slam/glim_supervisor/change_state", request).success

    def test_lifecycle_mirrors_and_drives_the_controller(self):
        from lifecycle_msgs.msg import Transition
        self.start()
        self.wait_state("active", 60)   # requires the ROS ready probe to see /slam/glim_ros
        self.wait_for(lambda: self.lifecycle_state() == "active", "lifecycle active")
        self.assertTrue(self.change(Transition.TRANSITION_DEACTIVATE))
        self.assertEqual(self.state()["state"], "standby")
        self.assertEqual(self.record(self.sessions()[-1])["outcome"], "saved")
        self.assertTrue(self.change(Transition.TRANSITION_ACTIVATE))
        self.wait_state("active", 60)
        self.assertEqual(len(self.sessions()), 2)
        self.assertEqual(self.cli("standby")[0], 0)   # operator path -> lifecycle follows
        self.wait_for(lambda: self.lifecycle_state() == "inactive", "lifecycle inactive")
        self.assertEqual(self.stop(), 0)

    def test_failed_lifecycle_activate_leaves_the_node_inactive(self):
        from lifecycle_msgs.msg import Transition
        self.seed.write_text("standby\n")
        self.control.write_text("crash")
        self.start()
        # standby leaves it unconfigured: a lifecycle manager's own bringup must work
        self.wait_for(lambda: self.lifecycle_state() == "unconfigured", "lifecycle unconfigured")
        self.assertTrue(self.change(Transition.TRANSITION_CONFIGURE))
        self.assertFalse(self.change(Transition.TRANSITION_ACTIVATE))
        time.sleep(3)   # several supervise ticks: nothing may re-activate it
        self.assertEqual(self.lifecycle_state(), "inactive")
        self.assertEqual(self.state()["target"], "standby")
        self.assertEqual(self.stop(), 0)

    def test_diagnostics_report_every_component(self):
        import rclpy
        from diagnostic_msgs.msg import DiagnosticArray
        latest = {}

        def collect(message):   # one publication = the complete component set
            latest.clear()
            latest.update({s.name: {kv.key: kv.value for kv in s.values} for s in message.status})
        self.node.create_subscription(DiagnosticArray, "/diagnostics", collect, 10)
        self.start()
        self.wait_state("active", 60)

        def settled():
            return (latest.get("slam: service", {}).get("op.state") == "active" and
                    float(latest.get("slam: odometry", {}).get("fps.delivered", 0)) > 5)
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline and not settled():
            rclpy.spin_once(self.node, timeout_sec=0.2)
        received = dict(latest)
        self.assertEqual(set(received), {f"slam: {c}" for c in
                                         ("service", "odometry", "imu", "points", "storage")})
        service = received["slam: service"]
        self.assertEqual((service["op.state"], service["health.vehicle_id"],
                          service["health.service"]), ("active", "7", "glim"))
        self.assertGreater(float(received["slam: odometry"]["fps.delivered"]), 5)
        self.assertEqual(self.stop(), 0)


if __name__ == "__main__":
    unittest.main()
