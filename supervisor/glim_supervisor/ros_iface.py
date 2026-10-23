"""ROS side of the supervisor: a lifecycle facade for mission software, and fleet /diagnostics.

/<ns>/glim_supervisor is a managed (lifecycle) node whose states mirror the controller's TARGET:
  inactive <-> standby, active <-> active.
A mission layer may `activate` / `deactivate` it with the standard lifecycle services; those
transitions run through the same controller as `rig activate` / `rig standby`, so both views
always agree. The supervise loop calls sync() to carry operator (socket) transitions back into
the lifecycle state. `shutdown` parks GLIM like deactivate and retires the lifecycle interface
until the container restarts.

It also measures GLIM's odometry output and the IMU input it consumes, counts publishers on the
point cloud topic (graph only — never a second subscriber to the largest stream on the vehicle),
and publishes diagnostic_msgs/DiagnosticArray on /diagnostics every health.interval_s.
"""
import logging
import secrets
import threading
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNodeMixin, TransitionCallbackReturn
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from . import health, storage

LOG = logging.getLogger("glim-supervisor")


def _resolve(namespace, topic):
    return topic if topic.startswith("/") else f"{namespace.rstrip('/')}/{topic}"


class SupervisorNode(LifecycleNodeMixin, Node):
    def __init__(self, controller, instance, identity, map_root):
        # LifecycleNode would put the lifecycle services in the node's default (mutually
        # exclusive) group: a change_state that blocks for minutes on a map save would then stall
        # get_state and every other query. rcl itself rejects overlapping transitions.
        Node.__init__(self, "glim_supervisor", namespace=instance["namespace"])
        LifecycleNodeMixin.__init__(self, enable_communication_interface=True,
                                    callback_group=ReentrantCallbackGroup())
        self.controller = controller
        self.instance = instance
        self.identity = identity
        self.map_root = map_root
        self.settings = instance["health"]
        self.namespace_ = instance["namespace"]
        self.topics = {"points": _resolve(self.namespace_, instance["inputs"]["points"]),
                       "imu": _resolve(self.namespace_, instance["inputs"]["imu"])}
        self.publisher_id = secrets.token_hex(8)
        self.sequence = 0
        self.sync_thread = None   # thread id while sync() drives a transition itself
        self.odometry = health.RateTracker()
        self.imu = health.RateTracker()
        self.session_seen = None
        self.wall = Clock(clock_type=ClockType.SYSTEM_TIME)

        # Monitoring never shares a callback group with the lifecycle services: a deactivate can
        # block for minutes while GLIM saves its map, and health must keep reporting through it.
        monitors = MutuallyExclusiveCallbackGroup()
        imu_group = MutuallyExclusiveCallbackGroup()
        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, f"{self.namespace_.rstrip('/')}/glim_ros/odom",
                                 lambda _: self.odometry.record(time.monotonic()), best_effort,
                                 callback_group=monitors)
        self.create_subscription(Imu, self.topics["imu"],
                                 lambda _: self.imu.record(time.monotonic()),
                                 qos_profile_sensor_data, callback_group=imu_group)
        self.diagnostics = self.create_publisher(
            DiagnosticArray, "/diagnostics",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.create_timer(self.settings["interval_s"], self.publish_health, callback_group=monitors)

    # --- lifecycle -----------------------------------------------------------------------
    def _external(self):
        return threading.get_ident() != self.sync_thread

    def _drive(self, target):
        try:
            # A lifecycle activate that reports FAILURE must leave the node inactive, so it may
            # not leave an `active` target behind for sync() to re-activate.
            self.controller.transition(target, revert_on_failure=(target == "active"))
            return TransitionCallbackReturn.SUCCESS
        except Exception as error:
            LOG.error("lifecycle -> %s failed: %s", target, error)
            return TransitionCallbackReturn.FAILURE

    def on_configure(self, state):
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        return self._drive("active") if self._external() else TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        return self._drive("standby") if self._external() else TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        if self._external() and self.controller.target == "active":
            self._drive("standby")
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state):
        return TransitionCallbackReturn.SUCCESS

    def lifecycle_label(self):
        try:
            return self._state_machine.current_state[1]
        except Exception:
            return None

    def sync(self):
        """Carry the controller's target into the lifecycle state (never the other way)."""
        label = self.lifecycle_label()
        if label in (None, "finalized") or self.controller.changing.is_set():
            return
        want = self.controller.target
        self.sync_thread = threading.get_ident()
        try:
            if label == "unconfigured":
                self.trigger_configure()
                label = self.lifecycle_label()
            if want == "active" and label == "inactive":
                self.trigger_activate()
            elif want == "standby" and label == "active":
                self.trigger_deactivate()
        except Exception as error:  # a concurrent ROS-side transition; converge next tick
            LOG.debug("lifecycle sync deferred: %s", error)
        finally:
            self.sync_thread = None

    # --- graph / health ------------------------------------------------------------------
    def glim_visible(self):
        try:
            return ("glim_ros", self.namespace_) in self.get_node_names_and_namespaces()
        except Exception:
            return None

    def publish_health(self):
        snapshot = self.controller.snapshot()
        if snapshot["session"] != self.session_seen:   # rates are per session
            self.session_seen = snapshot["session"]
            self.odometry.reset()
        try:
            points = self.count_publishers(self.topics["points"])
        except Exception:
            points = None
        try:
            root = storage.instance_root(self.map_root, self.identity["instance"])
            probe = next((p for p in (root, *root.parents) if p.exists()), self.map_root)
            disk = storage.disk_free(probe)
        except OSError as error:
            disk = str(error)
        now = time.monotonic()
        self.sequence += 1
        statuses = health.build_statuses(
            identity=self.identity, settings=self.settings, snapshot=snapshot,
            odometry=self.odometry.snapshot(now), imu=self.imu.snapshot(now),
            points_publishers=points, storage=disk, now=now, sequence=self.sequence,
            publisher_id=self.publisher_id, topics=self.topics)
        message = DiagnosticArray()
        message.header.stamp = self.wall.now().to_msg()
        for status in statuses:
            item = DiagnosticStatus()
            item.level = bytes([status["level"]])
            item.name = status["name"]
            item.message = status["message"]
            item.hardware_id = status["hardware_id"]
            item.values = [KeyValue(key=k, value=v) for k, v in status["values"].items()]
            message.status.append(item)
        self.diagnostics.publish(message)


class RosInterface:
    def __init__(self, controller, instance, identity, map_root):
        # The supervisor owns signal handling: SIGTERM must save the map, not kill the context.
        rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
        self.node = SupervisorNode(controller, instance, identity, map_root)
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, name="ros", daemon=True)

    def start(self):
        self.thread.start()

    def glim_visible(self):
        return self.node.glim_visible()

    def sync(self):
        self.node.sync()

    def close(self):
        try:
            self.executor.shutdown(timeout_sec=2)
            self.node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
