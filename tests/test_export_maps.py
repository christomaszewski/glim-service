"""tools/export_maps.py: rig's `export` verb for map sessions."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import export_maps  # noqa: E402


class ExportMapsTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.run_dir = self.root / "runs" / "r1"
        for session in ("20261023T010000Z", "20261023T020000Z-remap"):
            base = self.run_dir / "maps" / "slam" / session
            for rel in ("session.json", "map.ply", "evaluation.json", "config/config.json",
                        "logs/glim_odom.log", "dump/graph.bin", "dump/values.bin", "dump/graph.txt",
                        "dump/traj_imu.txt", "dump/000000/points_compact.bin"):
                (base / rel).parent.mkdir(parents=True, exist_ok=True)
                (base / rel).write_text(rel)
        self.dest = self.root / "export"

    def exported(self):
        base = self.dest / "maps" / "slam" / "20261023T010000Z"
        return sorted(str(p.relative_to(base)) for p in base.rglob("*") if p.is_file())

    def test_default_keeps_trajectories_and_the_ply_but_not_the_dump(self):
        options = export_maps.load_options(None)
        self.assertEqual(export_maps.export(self.run_dir, self.dest, "slam", options), (2, 14))
        self.assertEqual(self.exported(), ["config/config.json", "dump/graph.txt", "dump/traj_imu.txt",
                                           "evaluation.json", "logs/glim_odom.log", "map.ply",
                                           "session.json"])
        source = self.run_dir / "maps/slam/20261023T010000Z/map.ply"
        self.assertEqual(os.stat(source).st_ino,   # hardlinked: no disk cost on the vehicle
                         os.stat(self.dest / "maps/slam/20261023T010000Z/map.ply").st_ino)

    def test_full_and_none(self):
        export_maps.export(self.run_dir, self.dest, "slam", {"dump": "full", "logs": False})
        self.assertIn("dump/000000/points_compact.bin", self.exported())
        self.assertNotIn("logs/glim_odom.log", self.exported())

    def test_in_place_is_lossy_only_when_allowed(self):
        options = export_maps.load_options(None)
        with self.assertRaises(PermissionError):
            export_maps.export(self.run_dir, self.run_dir, "slam", options, in_place=True)
        self.assertEqual(export_maps.export(self.run_dir, self.run_dir, "slam",
                                            {"dump": "full", "logs": True}, in_place=True), (2, 0))
        sessions, dropped = export_maps.export(self.run_dir, self.run_dir, "slam", options,
                                               in_place=True, lossy=True)
        self.assertEqual((sessions, dropped), (2, 6))
        dump = self.run_dir / "maps/slam/20261023T010000Z/dump"
        self.assertEqual(sorted(p.name for p in dump.iterdir()), ["graph.txt", "traj_imu.txt"])

    def test_bad_options_and_the_rig_environment(self):
        bad = self.root / "options.yaml"
        bad.write_text("dump: some\n")
        with self.assertRaises(ValueError):
            export_maps.load_options(bad)
        options = self.root / "ok.yaml"
        options.write_text("dump: none\n")
        env = dict(os.environ, RIG_EXPORT_SOURCE=str(self.run_dir), RIG_EXPORT_DEST=str(self.dest),
                   RIG_EXPORT_OPTIONS=str(options), RIG_EXPORT_NAME="slam")
        result = subprocess.run([sys.executable, str(ROOT / "tools/export_maps.py")], env=env,
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, "")
        self.assertNotIn("dump/traj_imu.txt", self.exported())
        env["RIG_EXPORT_INPLACE"] = "1"
        env["RIG_EXPORT_DEST"] = str(self.run_dir)
        result = subprocess.run([sys.executable, str(ROOT / "tools/export_maps.py")], env=env,
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 3)
        self.assertIn("--lossy", result.stderr)

    def test_missing_maps_is_nothing_to_do(self):
        self.assertEqual(export_maps.export(self.root, self.dest, "other",
                                            export_maps.load_options(None)), (0, 0))


if __name__ == "__main__":
    unittest.main()
