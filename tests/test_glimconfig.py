"""Session config composition over upstream's commented JSON defaults."""
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "supervisor"))
sys.path.insert(0, str(ROOT / "tools"))

from glim_supervisor import glimconfig  # noqa: E402
import render_config  # noqa: E402
import yaml  # noqa: E402


def upstream_defaults():
    """The pinned upstream config: in the image, or a sibling glim checkout during development."""
    for candidate in (os.environ.get("GLIM_DEFAULTS_DIR"), "/opt/glim/share/glim/config",
                      ROOT.parent / "glim" / "config"):
        if candidate and (Path(candidate) / "config_ros.json").is_file():
            return Path(candidate)
    return None


def example_instance(**changes):
    cfg = yaml.safe_load((ROOT / "autonomy/glim.example.yaml").read_text())
    cfg.update(changes)
    return render_config.render_instance(cfg)


class StripCommentsTests(unittest.TestCase):
    def test_comments_outside_strings_only(self):
        text = '''{
          /*** block
           // nested line comment marker inside a block ***/
          "url": "http://example.com/a//b", // trailing
          "glob": "/*not a comment*/",
          "escaped": "quote \\" // still string"
        }'''
        self.assertEqual(json.loads(glimconfig.strip_comments(text)),
                         {"url": "http://example.com/a//b", "glob": "/*not a comment*/",
                          "escaped": 'quote " // still string'})

    def test_unterminated_block_comment(self):
        with self.assertRaises(ValueError):
            glimconfig.strip_comments('{"a": 1 /* oops')


@unittest.skipIf(upstream_defaults() is None, "upstream GLIM config not available")
class ComposeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.session = Path(self.temp.name) / "20261022T000000Z"
        self.defaults = upstream_defaults()

    def load(self, config_dir, name):
        return json.loads((config_dir / name).read_text())

    def test_every_upstream_default_parses(self):
        for path in sorted(self.defaults.glob("*.json")):
            with self.subTest(file=path.name):
                self.assertIsInstance(glimconfig.load_jsonc(path), dict)

    def test_lightweight_pipeline(self):
        config_dir, warnings = glimconfig.compose(self.defaults, example_instance(), self.session)
        self.assertEqual(warnings, [])
        top = self.load(config_dir, "config.json")["global"]
        self.assertEqual(top["config_odometry"], "config_odometry_cpu.json")
        self.assertEqual(top["config_sub_mapping"], "config_sub_mapping_passthrough.json")
        self.assertEqual(top["config_global_mapping"], "config_global_mapping_pose_graph.json")
        for name in top.values():
            if name:
                self.assertTrue((config_dir / name).is_file(), name)
        ros = self.load(config_dir, "config_ros.json")["glim_ros"]
        self.assertEqual(ros["points_topic"], "/top/points")
        self.assertEqual(ros["imu_qos"], {"profile": "sensor_data", "depth": 1000})
        self.assertTrue(ros["enable_local_mapping"] and ros["enable_global_mapping"])
        self.assertEqual(ros["acc_scale"], 0.0)  # an upstream default survives
        sensors = self.load(config_dir, "config_sensors.json")["sensors"]
        self.assertEqual(sensors["T_lidar_imu"], [0.0, 0.0, -0.1, 0.0, 0.0, 0.0, 1.0])
        logging = self.load(config_dir, "config_logging.json")["logging"]
        self.assertEqual(logging["log_dir"], str(self.session / "logs"))
        odometry = self.load(config_dir, "config_odometry_cpu.json")["odometry_estimation"]
        self.assertEqual(odometry["so_name"], "libodometry_estimation_cpu.so")

    def test_pipelines_select_modules(self):
        for pipeline, sub, local in (("cpu", "config_sub_mapping_cpu.json", True),
                                     ("odometry", "config_sub_mapping_passthrough.json", False)):
            with self.subTest(pipeline=pipeline):
                config_dir, _ = glimconfig.compose(self.defaults, example_instance(pipeline=pipeline),
                                                   self.session / pipeline)
                self.assertEqual(self.load(config_dir, "config.json")["global"]["config_sub_mapping"], sub)
                ros = self.load(config_dir, "config_ros.json")["glim_ros"]
                self.assertEqual((ros["enable_local_mapping"], ros["enable_global_mapping"]),
                                 (local, local))

    def test_unknown_keys_are_applied_and_reported(self):
        instance = example_instance(glim={"odometry": {"ivox_resolutoin": 0.5, "num_threads": 3}})
        config_dir, warnings = glimconfig.compose(self.defaults, instance, self.session)
        odometry = self.load(config_dir, "config_odometry_cpu.json")["odometry_estimation"]
        self.assertEqual((odometry["ivox_resolutoin"], odometry["num_threads"]), (0.5, 3))
        self.assertEqual(len(warnings), 1)
        self.assertIn("glim.odometry.ivox_resolutoin", warnings[0])

    def test_map_export_module_and_settings(self):
        config_dir, warnings = glimconfig.compose(self.defaults, example_instance(), self.session)
        document = self.load(config_dir, "config_ros.json")
        self.assertEqual(document["glim_ros"]["extension_modules"],
                         ["librviz_viewer.so", "libmap_export_monitor.so"])
        self.assertEqual(document["glim_service"], {"export_ply": True, "export_voxel_m": 0.1})
        self.assertTrue(document["glim_ros"]["publish_tf"])
        self.assertEqual(warnings, [])   # wrapper-owned keys are not "unknown"
        config_dir, _ = glimconfig.compose(self.defaults, example_instance(pipeline="odometry"),
                                           self.session / "odo")
        self.assertNotIn("libmap_export_monitor.so",   # no global map, nothing to export
                         self.load(config_dir, "config_ros.json")["glim_ros"]["extension_modules"])

    def test_unknown_pipeline(self):
        instance = example_instance()
        instance["pipeline"] = "gpu"
        with self.assertRaises(ValueError):
            glimconfig.compose(self.defaults, instance, self.session)


if __name__ == "__main__":
    unittest.main()
