"""GLIM's own input warnings: classification and the per-session tally."""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "supervisor"))

from glim_supervisor import glimlog  # noqa: E402


class Clock:
    now = 100.0

    def __call__(self):
        return self.now


class GlimLogTests(unittest.TestCase):
    def test_glims_literal_messages_are_classified(self):
        cases = {
            "[2026-10-23 03:16:52.632] [glim] [warning] large time difference between points and imu!!": "time_sync",
            "\x1b[33m[warning]\x1b[0m IMU timestamp rewind detected!!": "imu_time",
            "[odom] [warning] large time gap between consecutive IMU data!!": "imu_gap",
            "[glim] [warning] unexpected acceleration norm 0.98. assuming [m/s^2]": "imu_units",
            "[glim] [warning] failed to extract points from message": "point_format",
            "[glim] [error] negative per-point timestamp is found after time conversion!!": "point_time",
            "[glim] [warning] skip an invalid point cloud (stamp=12.0)": "lidar_time",
        }
        for line, category in cases.items():
            with self.subTest(line=line):
                self.assertEqual(glimlog.classify(line), category)
        for line in ("[glim] [info] load libodometry_estimation_cpu.so",
                     "[odom] [info] initial IMU state estimation result"):
            self.assertIsNone(glimlog.classify(line))
        self.assertEqual(set(glimlog.HINTS), {c for _, c in glimlog.PATTERNS})

    def test_tally_counts_ages_and_resets(self):
        clock = Clock()
        warnings = glimlog.InputWarnings(clock)
        warnings.observe("[glim] [info] nothing to see")
        for _ in range(3):
            warnings.observe("large time difference between points and imu!!")
        clock.now = 104.0
        seen = warnings.snapshot()
        self.assertEqual(list(seen), ["time_sync"])
        self.assertEqual((seen["time_sync"]["count"], seen["time_sync"]["age_s"]), (3, 4.0))
        self.assertIn("one clock", seen["time_sync"]["hint"])
        warnings.reset()
        self.assertEqual(warnings.snapshot(), {})


if __name__ == "__main__":
    unittest.main()
