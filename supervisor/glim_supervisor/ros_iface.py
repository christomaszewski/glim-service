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
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSPresetProfiles, QoSProfile,
                       ReliabilityPolicy)
from rclpy.signals import SignalHandlerOptions

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from . import health, storage

LOG = logging.getLogger("glim-supervisor")


def _resolve(namespace, topic):
    return topic if topic.startswith("/") else f"{namespace.rstrip('/')}/{topic}"


def glim_qos(spec):
    """The QoS GLIM's own subscription uses for a rendered `*_qos` block (glim_ros2 ros_qos.cpp:
    a profile, default sensor_data, with any of depth/reliability/durability/history applied).
    The monitor must match it, or it could report a stream GLIM itself cannot receive."""
    spec = spec or {}
    qos = QoSPresetProfiles.get_from_short_key(spec.get("profile", "sensor_data"))
    qos = QoSProfile(history=qos.history, depth=qos.depth, reliability=qos.reliability,
                     durability=qos.durability)
    if "depth" in spec:
        qos.depth = spec["depth"]
    for key, policy in (("reliability", ReliabilityPolicy), ("durability", DurabilityPolicy),
                        ("history", HistoryPolicy)):
        if key in spec:
            setattr(qos, key, policy.get_from_short_key(spec[key]))
    return qos


class Throttled:
    """Log a recurring failure once per interval, not once per heartbeat."""

    def __init__(self, interval_s=30.0):
        self.interval_s = interval_s
        self.last = {}

    def error(self, key, message, *args):
        now = time.monotonic()
        if now - self.last.get(key, -self.interval_s) >= self.interval_s:
            self.last[key] = now
            LOG.error(message, *args)


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
                                 glim_qos(instance["glim"]["glim_ros"].get("imu_qos")),
                                 callback_group=imu_group)
        self.throttled = Throttled()
        self.diagnostics = self.create_publisher(
            DiagnosticArray, "/diagnostics",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.create_timer(self.settings["interval_s"], self.publish_health, callback_group=monitors)

    # --- lifecycle -----------------------------------------------------------------------
    def _external(self):
        return threading.get_ident() != self.sync_thread

    def _drive(self, target):
        """Run a mission-requested transition; report what the controller actually did.

        The result follows the controller's resulting state, not whether transition() raised: a
        standby whose map save failed has still stopped GLIM (inactive is the truth), while a
        request that could not get the transition lock changed nothing. A failed activate reverts
        the target, so FAILURE leaves the node inactive with nothing left to re-activate it.
        """
        try:
            self.controller.transition(target, revert_on_failure=(target == "active"))
        except Exception as error:
            LOG.error("lifecycle -> %s: %s", target, error)
        if target == "active":
            reached = self.controller.target == "active"
        else:
            reached = self.controller.target == "standby" and self.controller.session is None
        return TransitionCallbackReturn.SUCCESS if reached else TransitionCallbackReturn.FAILURE

    def on_configure(self, state):
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        return self._drive("active") if self._external() else TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        return self._drive("standby") if self._external() else TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        # Finalizing retires this interface; refuse while GLIM would keep mapping unsupervised by it.
        if self._external() and self.controller.target == "active":
            return self._drive("standby")
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state):
        return TransitionCallbackReturn.SUCCESS

    def lifecycle_label(self):
        try:
            return self._state_machine.current_state[1]
        except Exception:
            return None

    def sync(self):
        """Carry the controller's target into the lifecycle state (never the other way).

        Only `active` is forced. Standby is either `unconfigured` or `inactive`, so a standard
        lifecycle manager's bringup (configure, then activate) and its cleanup both work as usual.
        """
        label = self.lifecycle_label()
        if label in (None, "finalized") or self.controller.changing.is_set():
            return
        want = self.controller.target
        self.sync_thread = threading.get_ident()
        try:
            if want == "active" and label == "unconfigured":
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
        try:
            self._publish_health()
        except Exception as error:   # one bad heartbeat must not end health reporting
            self.throttled.error("health", "health publication failed: %s", error)

    def _publish_health(self):
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
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self._spin, name="ros", daemon=True)

    def _spin(self):
        # executor.spin() re-raises the first callback exception and returns for good — the
        # lifecycle services and /diagnostics would silently die. Spin in steps and keep going.
        while not self.stopping.is_set():
            try:
                self.executor.spin_once(timeout_sec=0.5)
            except Exception as error:
                if self.stopping.is_set():
                    break
                self.node.throttled.error("spin", "ROS callback failed: %s", error)

    def start(self):
        self.thread.start()

    def glim_visible(self):
        return self.node.glim_visible()

    def sync(self):
        self.node.sync()

    def close(self, timeout_s=5.0):
        """Bounded teardown: it runs inside the container's stop grace period, after the map save,
        and a callback stuck on a hung volume must not turn a clean stop into a SIGKILL."""
        def teardown():
            try:
                self.executor.shutdown(timeout_sec=2, wait_for_threads=False)
                self.thread.join(timeout=2)
                self.node.destroy_node()
            finally:
                if rclpy.ok():
                    rclpy.shutdown()
        self.stopping.set()
        worker = threading.Thread(target=teardown, name="ros-close", daemon=True)
        worker.start()
        worker.join(timeout_s)
        if worker.is_alive():
            LOG.warning("ROS teardown still blocked after %.0fs; exiting without it", timeout_s)
