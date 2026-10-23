"""Fleet health statuses: levels, availability, limits and the reserved metadata."""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "supervisor"))

from glim_supervisor import health  # noqa: E402

IDENTITY = {"vehicle_id": "7", "instance": "slam", "service": "glim"}
SETTINGS = {"interval_s": 1.0, "stale_after_s": 5.0, "stall_timeout_s": 5.0, "limits": {}}
TOPICS = {"points": "/top/points", "imu": "/ins_main/imu/data"}


def snapshot(**changes):
    base = {"state": "active", "target": "active", "detail": "session S, up 60s", "restarts": 0,
            "failures": 0, "error": "", "session": "S", "session_dir": "/data/maps/slam/S",
            "session_uptime_s": 60.0, "last_session": None, "config_warnings": [],
            "pipeline": "lightweight"}
    base.update(changes)
    return base


def stream(fps, last):
    return {"fps": fps, "last": last, "count": 1}


class HealthTests(unittest.TestCase):
    def build(self, settings=SETTINGS, **kwargs):
        args = dict(identity=IDENTITY, settings=settings, snapshot=snapshot(),
                    odometry=stream(10.0, 99.9), imu=stream(200.0, 99.99), points_publishers=1,
                    storage=(120.0, 60.0), now=100.0, sequence=3, publisher_id="abc",
                    topics=TOPICS)
        args.update(kwargs)
        return {s["name"].split(": ", 1)[1]: s for s in health.build_statuses(**args)}

    def test_healthy_active_instance(self):
        statuses = self.build()
        self.assertEqual(set(statuses), {"service", "odometry", "imu", "points", "storage"})
        for status in statuses.values():
            self.assertEqual(status["level"], health.OK, status)
            values = status["values"]
            self.assertEqual(values["health.vehicle_id"], "7")
            self.assertEqual(values["health.instance"], "slam")
            self.assertEqual(values["health.service"], "glim")
            self.assertEqual(values["health.sequence"], "3")
            self.assertEqual(values["health.publish_interval_s"], "1.000")
            self.assertTrue(all(isinstance(v, str) for v in values.values()))
        self.assertEqual(statuses["odometry"]["values"]["fps.delivered"], "10.000")
        self.assertEqual(statuses["service"]["values"]["op.state"], "active")

    def test_confirmed_standby_is_healthy_and_streams_are_paused(self):
        statuses = self.build(snapshot=snapshot(state="standby", target="standby", session="",
                                                session_uptime_s=None),
                              odometry=stream(0.0, None), imu=stream(0.0, None), points_publishers=0)
        for name in ("service", "odometry", "imu", "points"):
            self.assertEqual(statuses[name]["level"], health.OK, name)
        for name in ("odometry", "imu", "points"):
            self.assertEqual(statuses[name]["values"]["health.availability"], "paused")
            self.assertNotIn("fps.delivered", statuses[name]["values"])  # no fabricated zero

    def test_stalls_warn_after_the_startup_grace(self):
        stalled = dict(odometry=stream(0.0, 90.0), imu=stream(0.0, None), points_publishers=0)
        statuses = self.build(**stalled)
        self.assertEqual(statuses["odometry"]["level"], health.WARN)
        self.assertIn("stalled for 10.0s", statuses["odometry"]["message"])
        self.assertEqual(statuses["imu"]["level"], health.WARN)
        self.assertEqual(statuses["imu"]["values"]["health.availability"], "unavailable")
        self.assertEqual(statuses["points"]["level"], health.WARN)
        self.assertIn("no publisher on /top/points", statuses["points"]["message"])
        starting = self.build(snapshot=snapshot(session_uptime_s=3.0), **stalled)
        self.assertEqual(starting["odometry"]["level"], health.OK)
        self.assertEqual(starting["imu"]["values"]["health.availability"], "retrying")

    def test_crash_loop_is_an_error_and_config_warnings_surface(self):
        statuses = self.build(snapshot=snapshot(state="transitioning", failures=3,
                                                error="GLIM exited unexpectedly (code -11)"))
        self.assertEqual(statuses["service"]["level"], health.ERROR)
        self.assertEqual(statuses["service"]["values"]["health.error"],
                         "GLIM exited unexpectedly (code -11)")
        statuses = self.build(snapshot=snapshot(config_warnings=["glim.odometry.x is not in ..."]))
        self.assertEqual(statuses["service"]["level"], health.WARN)
        self.assertIn("glim.odometry.x", statuses["service"]["message"])

    def test_incomplete_map_save_warns_even_in_standby(self):
        statuses = self.build(snapshot=snapshot(state="standby", target="standby", session="",
                                                save_error="session S: save_timeout (exit -9)"))
        self.assertEqual(statuses["service"]["level"], health.WARN)
        self.assertIn("last map save incomplete", statuses["service"]["message"])

    def test_producer_limits(self):
        settings = dict(SETTINGS, expected_odometry_hz=10.0, limits={
            "odometry": {"fps.delivered": {"warn_below": 8.0}},
            "storage": {"disk.free_gb": {"warn_below": 20.0, "error_below": 5.0}}})
        statuses = self.build(settings=settings, odometry=stream(6.0, 99.9), storage=(4.0, 2.0))
        self.assertEqual(statuses["odometry"]["level"], health.WARN)
        self.assertIn("fps.delivered 6.000 < 8.000", statuses["odometry"]["message"])
        self.assertEqual(statuses["odometry"]["values"]["fps.expected"], "10.000")
        self.assertEqual(statuses["storage"]["level"], health.ERROR)

    def test_unreadable_storage_is_reported_not_zeroed(self):
        statuses = self.build(storage="[Errno 5] Input/output error")
        self.assertEqual(statuses["storage"]["level"], health.WARN)
        self.assertNotIn("disk.free_gb", statuses["storage"]["values"])
        self.assertIn("Input/output", statuses["storage"]["values"]["health.error"])

    def test_rate_tracker(self):
        tracker = health.RateTracker(window_s=5.0)
        for i in range(11):
            tracker.record(100.0 + i * 0.1)
        snap = tracker.snapshot(101.0)
        self.assertAlmostEqual(snap["fps"], 10.0)
        self.assertEqual(snap["last"], 101.0)
        self.assertEqual(tracker.snapshot(200.0)["last"], None)  # window expired


if __name__ == "__main__":
    unittest.main()
