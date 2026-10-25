"""The supervisor's ROS layer against a fake controller (needs rclpy: runs inside the image)."""
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "supervisor"))

try:
    import rclpy
    from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from glim_supervisor import ros_iface
    HAVE_ROS = True
except ImportError:
    HAVE_ROS = False


def instance(imu_qos=None):
    return {"namespace": "/rostest", "instance": "rostest", "service": "glim",
            "inputs": {"points": "/top/points", "imu": "/rostest_imu"},
            "glim": {"glim_ros": {"imu_qos": imu_qos or {"profile": "sensor_data", "depth": 1000}}},
            "health": {"interval_s": 0.2, "stale_after_s": 5.0, "stall_timeout_s": 5.0, "limits": {}}}


class FakeController:
    def __init__(self, target="standby"):
        self.target = target
        self.session = None
        self.changing = threading.Event()
        self.on_transition = None
        self.snapshot_error = None

    def transition(self, target, revert_on_failure=False):
        if self.on_transition:
            return self.on_transition(target, revert_on_failure)
        self.target = target
        self.session = object() if target == "active" else None

    def snapshot(self):
        if self.snapshot_error:
            error, self.snapshot_error = self.snapshot_error, None
            raise error
        return {"state": self.target, "target": self.target, "detail": "fake", "restarts": 0,
                "failures": 0, "error": "", "save_error": "", "session": "",
                "session_dir": "", "session_uptime_s": None, "last_session": None,
                "config_warnings": [], "pipeline": "lightweight"}


@unittest.skipUnless(HAVE_ROS, "rclpy not available (runs inside the image in CI)")
class RosInterfaceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
        os.environ["ROS_DOMAIN_ID"] = str(200 + os.getpid() % 30)
        os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def node(self, controller=None, **kwargs):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        node = ros_iface.SupervisorNode(controller or FakeController(), instance(**kwargs),
                                        {"vehicle_id": "7", "instance": "rostest", "service": "glim"},
                                        Path(temp.name))
        self.addCleanup(node.destroy_node)
        return node

    def test_monitor_qos_matches_what_glim_subscribes_with(self):
        qos = ros_iface.glim_qos({"profile": "sensor_data", "depth": 1000})
        self.assertEqual((qos.reliability, qos.depth), (ReliabilityPolicy.BEST_EFFORT, 1000))
        qos = ros_iface.glim_qos({"reliability": "reliable"})   # profile defaults to sensor_data
        self.assertEqual((qos.reliability, qos.depth), (ReliabilityPolicy.RELIABLE, 5))
        qos = ros_iface.glim_qos({"profile": "default", "durability": "transient_local"})
        self.assertEqual((qos.reliability, qos.durability),
                         (ReliabilityPolicy.RELIABLE, DurabilityPolicy.TRANSIENT_LOCAL))

    def test_imu_monitor_sees_only_what_glim_could(self):
        from sensor_msgs.msg import Imu
        from rclpy.qos import qos_profile_sensor_data
        received = {}
        for name, spec in (("compatible", None), ("reliable", {"reliability": "reliable"})):
            node = self.node(imu_qos=spec)
            publisher = rclpy.create_node(f"imu_pub_{name}")
            self.addCleanup(publisher.destroy_node)
            pub = publisher.create_publisher(Imu, "/rostest_imu", qos_profile_sensor_data)
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                pub.publish(Imu())
                rclpy.spin_once(node, timeout_sec=0.05)
            received[name] = node.imu.count
            node.destroy_node()
        self.assertGreater(received["compatible"], 0)
        self.assertEqual(received["reliable"], 0)   # best-effort publisher, reliable reader

    def test_a_failing_heartbeat_does_not_end_health_reporting(self):
        controller = FakeController()
        node = self.node(controller)
        controller.snapshot_error = RuntimeError("boom")
        node.publish_health()          # swallowed and logged, not raised into the executor
        node.publish_health()
        self.assertEqual(node.sequence, 1)

    def test_spin_survives_callback_exceptions(self):
        interface = ros_iface.RosInterface.__new__(ros_iface.RosInterface)
        interface.stopping = threading.Event()
        interface.node = type("N", (), {"throttled": ros_iface.Throttled()})()
        calls = []

        class Executor:
            def spin_once(self, timeout_sec=None):
                calls.append(1)
                if len(calls) == 1:
                    raise RuntimeError("callback failed")
                if len(calls) == 3:
                    interface.stopping.set()
        interface.executor = Executor()
        interface._spin()
        self.assertEqual(len(calls), 3)

    def test_lifecycle_results_follow_the_controllers_outcome(self):
        from rclpy.lifecycle import TransitionCallbackReturn as R
        controller = FakeController()
        node = self.node(controller)

        def failed_activate(target, revert):
            self.assertTrue(revert)
            raise RuntimeError("GLIM exited during startup")    # target reverted: stays standby
        controller.on_transition = failed_activate
        self.assertEqual(node._drive("active"), R.FAILURE)

        def standby_unsaved(target, revert):
            controller.target, controller.session = "standby", None
            raise RuntimeError("map was not saved cleanly")       # but GLIM did stop
        controller.target, controller.session = "active", object()
        controller.on_transition = standby_unsaved
        self.assertEqual(node._drive("standby"), R.SUCCESS)

        def busy(target, revert):
            raise RuntimeError("transition in progress")           # nothing changed
        controller.target, controller.session = "active", object()
        controller.on_transition = busy
        self.assertEqual(node._drive("standby"), R.FAILURE)
        self.assertEqual(node._drive("active"), R.SUCCESS)

    def test_sync_leaves_unconfigured_alone_but_forces_active(self):
        controller = FakeController("standby")
        node = self.node(controller)
        node.sync()
        self.assertEqual(node.lifecycle_label(), "unconfigured")   # a manager may configure it
        controller.target = "active"
        node.sync()
        self.assertEqual(node.lifecycle_label(), "active")
        controller.target = "standby"
        node.sync()
        self.assertEqual(node.lifecycle_label(), "inactive")


if __name__ == "__main__":
    unittest.main()
