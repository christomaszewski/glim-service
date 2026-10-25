"""tools/render_config.py: validation, derived GLIM keys, determinism, env output."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import render_config as rc  # noqa: E402

EXAMPLE = ROOT / "autonomy" / "glim.example.yaml"


def example():
    return yaml.safe_load(EXAMPLE.read_text())


class RenderInstanceTests(unittest.TestCase):
    def render(self, cfg, sim_time=None):
        return rc.render_instance(cfg, sim_time)

    def assertRejected(self, cfg, fragment):
        with self.assertRaises(rc.ConfigError) as caught:
            rc.render_env(cfg, warn=False)
            self.render(cfg)
        self.assertIn(fragment, str(caught.exception))

    def test_example_renders_wired_to_ouster_and_sbg(self):
        out = self.render(example())
        self.assertEqual(out["instance"], "slam")
        self.assertEqual(out["namespace"], "/slam")
        self.assertEqual(out["pipeline"], "lightweight")
        ros = out["glim"]["glim_ros"]
        self.assertEqual(ros["points_topic"], "/top/points")
        self.assertEqual(ros["imu_topic"], "/ins_main/imu/data")
        self.assertEqual(ros["imu_qos"], {"profile": "sensor_data", "depth": 1000})
        self.assertEqual(ros["extension_modules"], ["librviz_viewer.so"])
        self.assertFalse(ros["publish_imu2lidar"])
        self.assertEqual(out["glim"]["sensors"]["T_lidar_imu"], [0.0, 0.0, -0.1, 0.0, 0.0, 0.0, 1.0])
        self.assertEqual(out["glim"]["preprocess"]["distance_near_thresh"], 1.0)
        self.assertEqual(out["map"]["save_timeout_s"], 240.0)
        self.assertFalse(out["use_sim_time"])

    def test_sim_time_follows_rig_unless_config_is_explicit(self):
        cfg = example()
        self.assertTrue(self.render(cfg, "1")["use_sim_time"])
        self.assertFalse(self.render(cfg, "")["use_sim_time"])
        cfg["ros"] = {"use_sim_time": False}
        self.assertFalse(self.render(cfg, "1")["use_sim_time"])
        cfg["ros"] = {"use_sim_time": True}
        self.assertTrue(self.render(cfg, None)["use_sim_time"])
        with self.assertRaises(rc.ConfigError):
            self.render(example(), "maybe")

    def test_extrinsic_is_required_and_must_be_a_unit_quaternion(self):
        cfg = example()
        del cfg["extrinsics"]
        self.assertRejected(cfg, "extrinsics.T_lidar_imu is required")
        cfg = example()
        cfg["extrinsics"]["T_lidar_imu"] = [0, 0, 0, 0, 0, 90]  # Euler angles by mistake
        self.assertRejected(cfg, "[x, y, z, qx, qy, qz, qw]")
        cfg["extrinsics"]["T_lidar_imu"] = [0, 0, 0, 0, 0, 0.5, 0.5]
        self.assertRejected(cfg, "unit quaternion")
        cfg["extrinsics"]["T_lidar_imu"] = [0, 0, 0, 0, 0, 0, 1.0004]  # rounding noise: normalized
        self.assertEqual(self.render(cfg)["glim"]["sensors"]["T_lidar_imu"][6], 1.0)

    def test_derived_keys_have_one_spelling(self):
        for module, key, owner in (("ros", "imu_topic", "inputs.imu.topic"),
                                   ("ros", "base_frame_id", "frames.base"),
                                   ("ros", "enable_global_mapping", "pipeline"),
                                   ("sensors", "T_lidar_imu", "extrinsics.T_lidar_imu"),
                                   ("odometry", "so_name", "pipeline")):
            with self.subTest(key=key):
                cfg = example()
                cfg["glim"].setdefault(module, {})[key] = "x"
                self.assertRejected(cfg, owner)

    def test_typos_fail_loudly(self):
        for path, fragment in ((("pipline",), "unknown top-level key"),
                               (("glim", "odometery"), "unknown glim key"),
                               (("inputs", "point"), "unknown inputs key"),
                               (("frames", "bse"), "unknown frames key"),
                               (("health", "limits", "odom"), "unknown health.limits key")):
            with self.subTest(path=path):
                cfg = example()
                block = cfg
                for key in path[:-1]:
                    block = block.setdefault(key, {})
                block[path[-1]] = {}
                self.assertRejected(cfg, fragment)

    def test_inputs_and_qos_are_validated(self):
        cfg = example()
        del cfg["inputs"]["imu"]
        self.assertRejected(cfg, "inputs.imu.topic is required")
        cfg = example()
        cfg["inputs"]["points"]["topic"] = "/top/points/"
        self.assertRejected(cfg, "not a ROS topic name")
        cfg = example()
        cfg["inputs"]["points"]["qos"] = {"reliability": "reliabel"}
        self.assertRejected(cfg, "reliability")
        cfg["inputs"]["points"]["qos"] = "sensor_data"
        self.assertEqual(self.render(cfg)["glim"]["glim_ros"]["points_qos"], {"profile": "sensor_data"})
        cfg["inputs"]["points"]["qos"] = {"profile": "default", "depth": 5, "reliability": "reliable"}
        self.assertEqual(self.render(cfg)["glim"]["glim_ros"]["points_qos"],
                         {"profile": "default", "depth": 5, "reliability": "reliable"})
        # a partial override keeps the IMU's deep queue (GLIM treats any key as a full spec)
        cfg["inputs"]["imu"]["qos"] = {"reliability": "reliable"}
        self.assertEqual(self.render(cfg)["glim"]["glim_ros"]["imu_qos"],
                         {"profile": "sensor_data", "depth": 1000, "reliability": "reliable"})

    def test_frames(self):
        cfg = example()
        cfg["frames"]["odom"] = "map"
        self.assertRejected(cfg, "must differ")
        cfg = example()
        cfg["frames"]["base"] = "/base_link"
        self.assertRejected(cfg, "tf2 frame id")
        cfg = example()
        cfg["frames"].update(base="base_link", publish_imu_to_lidar=True)
        ros = self.render(cfg)["glim"]["glim_ros"]
        self.assertEqual((ros["base_frame_id"], ros["publish_imu2lidar"]), ("base_link", True))

    def test_identity_and_pipeline(self):
        cfg = example()
        cfg["name"] = "SLAM-1"
        self.assertRejected(cfg, "must match [a-z][a-z0-9_]*")
        cfg = example()
        cfg["service"] = "ouster"
        self.assertRejected(cfg, "only runs `glim` configs")
        cfg = example()
        cfg["pipeline"] = "gpu"
        with self.assertRaises(rc.ConfigError):
            self.render(cfg)
        cfg = example()
        cfg["name"] = "mapper"   # rig stamps the row's name; the namespace follows it
        self.assertEqual(self.render(cfg)["namespace"], "/mapper")
        cfg["ros"] = {"namespace": "slam"}
        self.assertEqual(self.render(cfg)["namespace"], "/slam")
        cfg["initial_state"] = "parked"
        self.assertRejected(cfg, "initial_state")

    def test_health_limits(self):
        cfg = example()
        cfg["health"]["limits"] = {"storage": {"disk.free_gb": {"warn_below": 20, "error_below": 5}}}
        limits = self.render(cfg)["health"]["limits"]
        self.assertEqual(limits, {"storage": {"disk.free_gb": {"warn_below": 20.0, "error_below": 5.0}}})
        cfg["health"]["limits"] = {"storage": {"disk.free_gb": {"warn_beneath": 20}}}
        self.assertRejected(cfg, "unknown health.limits.storage.disk.free_gb key")
        cfg["health"] = {"interval_s": 2, "stale_after_s": 3}
        self.assertRejected(cfg, "two health.interval_s periods")

    def test_extension_modules_without_ros_outputs_warn(self):
        cfg = example()
        cfg["glim"]["ros"] = {"extension_modules": []}
        with tempfile.TemporaryFile("w+") as stderr:
            saved, sys.stderr = sys.stderr, stderr
            try:
                out = self.render(cfg)
            finally:
                sys.stderr = saved
            stderr.seek(0)
            self.assertIn("librviz_viewer.so", stderr.read())
        self.assertEqual(out["glim"]["glim_ros"]["extension_modules"], [])


class TfAndEvaluationTests(unittest.TestCase):
    def test_extrinsic_from_tf_needs_named_frames(self):
        cfg = example()
        cfg["extrinsics"] = {"from_tf": True}
        with self.assertRaisesRegex(rc.ConfigError, "frames.imu and frames.lidar"):
            rc.render_instance(cfg)
        cfg["frames"].update(imu="imu_link", lidar="ouster_lidar")
        out = rc.render_instance(cfg)
        self.assertEqual(out["extrinsics"], {"source": "tf", "imu_frame": "imu_link",
                                             "lidar_frame": "ouster_lidar"})
        self.assertNotIn("T_lidar_imu", out["glim"]["sensors"])
        cfg["extrinsics"]["T_lidar_imu"] = [0, 0, 0, 0, 0, 0, 1]
        with self.assertRaisesRegex(rc.ConfigError, "not both"):
            rc.render_instance(cfg)

    def test_publish_tf_switch(self):
        cfg = example()
        self.assertTrue(rc.render_instance(cfg)["glim"]["glim_ros"]["publish_tf"])
        cfg["frames"]["publish_tf"] = False
        self.assertFalse(rc.render_instance(cfg)["glim"]["glim_ros"]["publish_tf"])
        cfg = example()
        cfg["glim"]["ros"] = {"publish_tf": False}
        with self.assertRaisesRegex(rc.ConfigError, "frames.publish_tf"):
            rc.render_instance(cfg)

    def test_evaluation_block(self):
        self.assertIsNone(rc.render_instance(example())["evaluation"])
        cfg = example()
        cfg["evaluation"] = {"reference_topic": "/ins_main/odom"}
        self.assertEqual(rc.render_instance(cfg)["evaluation"],
                         {"reference_topic": "/ins_main/odom", "max_dt_s": 0.05})
        cfg["evaluation"] = {"reference_topic": "not a topic!"}
        with self.assertRaises(rc.ConfigError):
            rc.render_instance(cfg)


class ResourceAndMapTests(unittest.TestCase):
    def test_limits_render_into_env_and_guard_into_the_instance(self):
        cfg = example()
        cfg["resources"] = {"memory_limit_gb": 6, "cpus": 4, "cpuset": "4-7", "memory_guard_pct": 85}
        env = dict(rc.render_env(cfg, warn=False))
        self.assertEqual((env["MEM_LIMIT"], env["CPUS"], env["CPUSET"]), ("6144m", "4", "4-7"))
        self.assertEqual(rc.render_instance(cfg)["resources"],
                         {"memory_guard_pct": 85.0, "memory_limit_gb": 6.0})
        env = dict(rc.render_env(example(), warn=False))
        self.assertEqual((env["MEM_LIMIT"], env["CPUS"], env["CPUSET"]), ("", "", ""))

    def test_resource_validation(self):
        for block, fragment in (({"cpuset": "four"}, "CPU list"), ({"memory_guard_pct": 100}, "below 100"),
                                ({"memory_guard_pct": 20}, ">= 50"), ({"memory_limit_gb": 0}, "positive"),
                                ({"swap": 1}, "unknown resources key")):
            with self.subTest(block=block):
                cfg = example()
                cfg["resources"] = block
                with self.assertRaises(rc.ConfigError) as caught:
                    rc.render_env(cfg, warn=False)
                self.assertIn(fragment, str(caught.exception))

    def test_map_guards_and_export_defaults(self):
        out = rc.render_instance(example())["map"]
        self.assertEqual(out, {"save_timeout_s": 240.0, "min_free_gb": 1.0, "keep_sessions": 0,
                               "export_ply": True, "export_voxel_m": 0.1})
        cfg = example()
        cfg["map"] = {"keep_sessions": -1}
        with self.assertRaises(rc.ConfigError):
            rc.render_instance(cfg)


class YamlTypeTests(unittest.TestCase):
    """YAML is generous with types (`no` is False, `2026-10-23` a date, `.inf` a float)."""

    def assertRejected(self, cfg, fragment):
        with self.assertRaises(rc.ConfigError) as caught:
            rc.render_env(cfg, warn=False)
            rc.render_instance(cfg)
        self.assertIn(fragment, str(caught.exception))

    def test_falsy_values_are_errors_not_defaults(self):
        for key, value, fragment in (("name", False, "name False"),
                                     ("initial_state", False, "initial_state False"),
                                     ("service", 0, "service 0")):
            with self.subTest(key=key):
                cfg = example()
                cfg[key] = value
                self.assertRejected(cfg, fragment)
        cfg = example()
        cfg["pipeline"] = None   # explicit null = the default
        self.assertEqual(rc.render_instance(cfg)["pipeline"], "lightweight")

    def test_frames_cannot_publish_a_transform_to_themselves(self):
        cfg = example()
        cfg["frames"]["base"] = "odom"
        self.assertRejected(cfg, "from a frame to itself")

    def test_odd_keys_are_clean_errors(self):
        cfg = example()
        cfg[7] = "x"
        cfg["extra"] = "y"   # mixed key types must not crash the message
        self.assertRejected(cfg, "unknown top-level key(s): 7, extra")
        cfg = example()
        cfg["glim"]["odometry"] = {1: 2}
        self.assertRejected(cfg, "must be a string")

    def test_lenient_identity_ignores_unrelated_breakage(self):
        cfg = example()
        cfg.update(inital_state="standby", zenoh={"shm_pool_mb": -1}, map={"save_timeout_s": "x"})
        with self.assertRaises(rc.ConfigError):
            rc.render_env(cfg, warn=False)
        env = dict(rc.render_env(cfg, warn=False, lenient=True))
        self.assertEqual((env["NAME"], env["STOP_GRACE_S"], env["ZENOH_OVERRIDE"]), ("slam", "270", ""))
        cfg["name"] = "Bad Name"   # identity itself is still enforced
        with self.assertRaises(rc.ConfigError):
            rc.render_env(cfg, warn=False, lenient=True)


class CommandLineTests(unittest.TestCase):
    def run_tool(self, *args, config=None):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "slam.yaml"
            path.write_text(yaml.safe_dump(config if config is not None else example()))
            return subprocess.run([sys.executable, str(ROOT / "tools/render_config.py"), *args,
                                   str(path)], capture_output=True, timeout=30)

    def test_env0_is_literal_nul_pairs(self):
        cfg = example()
        cfg["zenoh"] = {"shared_memory": True, "overrides": {"connect": {"endpoints": ["tcp/x:7447"]}}}
        result = self.run_tool("--env0", config=cfg)
        self.assertEqual(result.returncode, 0, result.stderr)
        fields = result.stdout.split(b"\0")
        pairs = dict(zip(fields[0::2], fields[1::2]))
        self.assertEqual(pairs[b"NAME"], b"slam")
        self.assertEqual(pairs[b"NAMESPACE"], b"/slam")
        self.assertEqual(pairs[b"INITIAL_STATE"], b"active")
        self.assertEqual(pairs[b"STOP_GRACE_S"], b"270")
        self.assertEqual(pairs[b"ZENOH_OVERRIDE"],
                         b'transport/shared_memory/enabled=true;connect/endpoints=["tcp/x:7447"]')

    def test_instance_output_is_deterministic_json(self):
        first = self.run_tool("--instance", "--sim-time", "1")
        second = self.run_tool("--instance", "--sim-time", "1")
        self.assertEqual(first.returncode, 0, first.stderr)
        self.assertEqual(first.stdout, second.stdout)
        self.assertTrue(json.loads(first.stdout)["use_sim_time"])

    def test_errors_go_to_stderr_with_the_config_path(self):
        cfg = example()
        del cfg["extrinsics"]
        result = self.run_tool("--instance", config=cfg)
        self.assertEqual(result.returncode, 2)
        self.assertEqual(result.stdout, b"")
        self.assertIn(b"slam.yaml: extrinsics.T_lidar_imu is required", result.stderr)

    def test_a_repeated_key_is_an_error_not_a_silent_override(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "slam.yaml"
            text = EXAMPLE.read_text().replace("  publish_tf: true", "  publish_tf: false\n  publish_tf: true")
            self.assertNotEqual(text, EXAMPLE.read_text())
            path.write_text(text)
            result = subprocess.run([sys.executable, str(ROOT / "tools/render_config.py"),
                                     "--instance", str(path)], capture_output=True, timeout=30)
        self.assertEqual(result.returncode, 2)
        self.assertEqual(result.stdout, b"")
        self.assertIn(b"duplicate key 'publish_tf'", result.stderr)

    def test_values_json_cannot_carry_exit_cleanly(self):
        for value in (float("inf"), float("nan")):
            with self.subTest(value=value):
                cfg = example()
                cfg["glim"]["odometry"] = {"smoother_lag": value}
                result = self.run_tool("--instance", config=cfg)
                self.assertEqual(result.returncode, 2)
                self.assertIn(b"unsupported value", result.stderr)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "slam.yaml"
            path.write_text(EXAMPLE.read_text() + "\n")
            path.write_text(path.read_text().replace("  preprocess:\n",
                                                     "  preprocess:\n    calibrated: 2026-10-23\n"))
            result = subprocess.run([sys.executable, str(ROOT / "tools/render_config.py"),
                                     "--instance", str(path)], capture_output=True, timeout=30)
            self.assertEqual(result.returncode, 2)
            self.assertIn(b"unsupported value", result.stderr)
            self.assertNotIn(b"Traceback", result.stderr)

    def test_lenient_flag(self):
        cfg = example()
        cfg["inital_state"] = "standby"
        self.assertEqual(self.run_tool("--env0", config=cfg).returncode, 2)
        self.assertEqual(self.run_tool("--env0", "--lenient", config=cfg).returncode, 0)

    def test_zenoh_warning_surfaces_once_per_launcher_run(self):
        cfg = example()
        cfg["zenoh"] = {"shm_pool_mb": 64}
        env = self.run_tool("--env0", config=cfg)
        instance = self.run_tool("--instance", config=cfg)
        self.assertIn(b"shm_pool_mb is ignored", env.stderr)
        self.assertNotIn(b"shm_pool_mb", instance.stderr)


if __name__ == "__main__":
    unittest.main()
