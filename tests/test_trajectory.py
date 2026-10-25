"""Trajectory scoring: association, Horn alignment, ATE."""
import json
import math
from pathlib import Path
import random
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "supervisor"))

from glim_supervisor import trajectory  # noqa: E402


def rotation_z(angle):
    c, s = math.cos(angle), math.sin(angle)
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def apply(r, t, p):
    return [sum(r[i][j] * p[j] for j in range(3)) + t[i] for i in range(3)]


def write_tum(path, rows):
    path.write_text("".join(f"{t:.6f} {x:.6f} {y:.6f} {z:.6f} 0 0 0 1\n" for t, x, y, z in rows))


class TrajectoryTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        random.seed(3)
        # one lap of a 100 x 60 m ellipse (~254 m), 10 Hz for 100 s
        self.ref = [(1000.0 + i * 0.1, 50 * math.cos(i / 160), 30 * math.sin(i / 160), 0.01 * i)
                    for i in range(1000)]

    def estimate(self, noise, time_shift=0.0, r=None, t=(5.0, -3.0, 1.0)):
        r = r or rotation_z(0.7)
        return [(ts + time_shift, *[v + random.gauss(0, noise) for v in apply(r, t, (x, y, z))])
                for ts, x, y, z in self.ref]

    def test_alignment_recovers_a_rigid_transform(self):
        source = [p[1:] for p in self.ref[:50]]
        target = [apply(rotation_z(-1.2), [2, 3, 4], p) for p in source]
        r, t = trajectory.align(source, target)
        for s, d in zip(source, target):
            self.assertLess(math.dist(apply(r, t, s), d), 1e-6)

    def test_ate_measures_the_noise_not_the_frame(self):
        write_tum(self.root / "ref.tum", self.ref)
        write_tum(self.root / "est.tum", self.estimate(noise=0.05))
        result = trajectory.evaluate(self.root / "est.tum", self.root / "ref.tum")
        self.assertEqual(result["matched"], 1000)
        self.assertAlmostEqual(result["ate_rmse_m"], 0.05 * math.sqrt(3), delta=0.02)
        self.assertAlmostEqual(result["reference_path_length_m"], 254.3, delta=1.0)
        self.assertLess(result["final_error_pct_of_path"], 0.1)

    def test_drift_shows_up(self):
        drifting = [(t, x + 0.01 * i, y, z) for i, (t, x, y, z) in enumerate(self.ref)]
        write_tum(self.root / "ref.tum", self.ref)
        write_tum(self.root / "est.tum", drifting)
        self.assertGreater(trajectory.evaluate(self.root / "est.tum", self.root / "ref.tum")["ate_max_m"], 1.0)

    def test_association_respects_max_dt_and_clock_mismatch_fails_clearly(self):
        write_tum(self.root / "ref.tum", self.ref)
        write_tum(self.root / "est.tum", self.estimate(noise=0.0, time_shift=0.03))
        self.assertEqual(trajectory.evaluate(self.root / "est.tum", self.root / "ref.tum", 0.05)["matched"], 1000)
        write_tum(self.root / "est.tum", self.estimate(noise=0.0, time_shift=3600.0))   # wrong clock
        with self.assertRaisesRegex(ValueError, "one clock"):
            trajectory.evaluate(self.root / "est.tum", self.root / "ref.tum")

    def test_command_line(self):
        write_tum(self.root / "ref.tum", self.ref)
        write_tum(self.root / "est.tum", self.estimate(noise=0.01))
        result = subprocess.run([sys.executable, str(ROOT / "tools/eval_trajectory.py"),
                                 str(self.root / "est.tum"), str(self.root / "ref.tum")],
                                capture_output=True, text=True, timeout=60)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertLess(json.loads(result.stdout)["ate_rmse_m"], 0.05)
        bad = subprocess.run([sys.executable, str(ROOT / "tools/eval_trajectory.py"),
                              str(self.root / "missing.tum"), str(self.root / "ref.tum")],
                             capture_output=True, text=True, timeout=60)
        self.assertEqual(bad.returncode, 1)
        self.assertIn("eval_trajectory:", bad.stderr)


if __name__ == "__main__":
    unittest.main()
