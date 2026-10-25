"""Fleet health statuses for one GLIM instance (ouster docs/SERVICE_HEALTH.md, revision 1).

Pure functions over observations, so the levels, metadata and limits are testable without ROS.
Components, each named "<instance>: <component>":
  service   operational state, restarts, config warnings (ERROR while GLIM keeps failing)
  odometry  GLIM's own output: fps.delivered, frame_age_s (WARN when stalled while active)
  imu       the IMU input as GLIM's subscriber sees it: fps.delivered, frame_age_s
  points    publishers on the point cloud topic (observed from the graph; no extra subscription
            to the vehicle's largest stream)
  storage   free space where map sessions are written: disk.free_gb, disk.free_pct
Operational states are values, never levels: a confirmed standby is healthy, and streams are
demanded whenever the target is active (including while GLIM restarts). A measurement that does
not exist is omitted, never reported as zero.
"""
from collections import deque
import math
import threading

OK, WARN, ERROR, STALE = 0, 1, 2, 3
STARTUP_GRACE_S = 15.0   # GLIM's initial state estimation needs a few seconds of IMU + scans
MIN_RATE_SPAN_S = 1.0    # observe at least this long before reporting a delivered rate


class RateTracker:
    """Arrival times on the monotonic clock -> delivered rate and age of the newest sample.

    The rate counts arrivals in the trailing window, so it decays when a stream stops instead of
    freezing at its last value; the newest arrival is kept separately, so a stall stays visible
    as a growing frame age. Thread-safe: subscriptions record
    on executor threads while the health timer reads.
    """

    def __init__(self, window_s=5.0, maxlen=8192):
        self.window_s = window_s
        self.arrivals = deque(maxlen=maxlen)
        self.count = 0
        self.last = None
        self.since = None
        self.lock = threading.Lock()

    def record(self, now):
        with self.lock:
            self.arrivals.append(now)
            self.count += 1
            self.last = now
            if self.since is None:
                self.since = now

    def snapshot(self, now):
        with self.lock:
            while self.arrivals and now - self.arrivals[0] > self.window_s:
                self.arrivals.popleft()
            n = len(self.arrivals)
            observed = None if self.since is None else now - self.since
            if observed is None or observed < MIN_RATE_SPAN_S:
                fps = None                       # nothing yet, or too little to call it a rate
            elif observed >= self.window_s:
                fps = n / self.window_s          # full window; a silent one is a real 0 Hz
            else:
                fps = (n - 1) / (now - self.since) if n >= 2 else None
            return {"fps": fps, "last": self.last, "count": self.count}

    def reset(self):
        with self.lock:
            self.arrivals.clear()
            self.last = None
            self.since = None


def _fmt(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        return f"{value:.3f}" if math.isfinite(value) else "nan"
    return str(value)


def apply_limits(level, messages, values, limits):
    """Producer thresholds (strict > / <; ERROR wins over WARN). Absent metrics are never zero."""
    for metric, bounds in (limits or {}).items():
        value = values.get(metric)
        if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
            continue
        for key, threshold in bounds.items():
            breached = value > threshold if key.endswith("_above") else value < threshold
            if not breached:
                continue
            severity = ERROR if key.startswith("error") else WARN
            level = max(level, severity)
            relation = ">" if key.endswith("_above") else "<"
            messages.append(f"{metric} {_fmt(float(value))} {relation} {_fmt(float(threshold))}")
    return level


def _stream(component, name, snapshot, now, demanded, starting, stall_timeout_s, limits,
            expected=None):
    if not demanded:
        return OK, f"{name}: standby (not demanded)", {}, "paused"
    values = {}
    if snapshot["fps"] is not None:
        values["fps.delivered"] = snapshot["fps"]
    if expected is not None:
        values["fps.expected"] = expected
    age = None if snapshot["last"] is None else max(0.0, now - snapshot["last"])
    if age is not None:
        values["frame_age_s"] = age
    messages = []
    level = OK
    availability = "current"
    if age is None:
        availability = "retrying" if starting else "unavailable"
        if not starting:
            level = WARN
        messages.append(f"{name}: none received yet")
    elif age > stall_timeout_s and not starting:
        level = WARN
        messages.append(f"{name}: stalled for {age:.1f}s")
    elif snapshot["fps"] is None:
        messages.append(f"{name}: receiving")
    else:
        messages.append(f"{name}: {snapshot['fps']:.1f} Hz")
    if not starting:   # thresholds judge a running stream, not its first seconds
        level = apply_limits(level, messages, values, limits.get(component))
    return level, "; ".join(messages), values, availability


def build_statuses(*, identity, settings, snapshot, odometry, imu, points_publishers, storage,
                   now, sequence, publisher_id, topics):
    """-> [{"level", "name", "message", "hardware_id", "values": {str: str}}], one per component.

    identity: {vehicle_id, instance, service}; settings: the instance file's `health` block;
    snapshot: Controller.snapshot(); odometry/imu: RateTracker snapshots; points_publishers: int or
    None (graph unavailable); storage: (free_gb, free_pct) or an error string; topics: {points, imu}.
    """
    limits = settings.get("limits") or {}
    stall = settings["stall_timeout_s"]
    demanded = snapshot["target"] == "active"
    running = bool(snapshot["session"])
    uptime = snapshot.get("session_uptime_s")
    # A first start (no failures yet) or a young session is still initializing; a session that is
    # missing after failures is a restart in backoff — demanded, and reported.
    starting = (uptime < STARTUP_GRACE_S) if uptime is not None else snapshot["failures"] == 0
    components = []

    # service ----------------------------------------------------------------------------
    level = OK
    messages = [f"{snapshot['state']}: {snapshot['detail']}"]
    if snapshot["failures"] >= 3:
        level = ERROR
        messages.append(f"GLIM keeps failing ({snapshot['failures']} consecutive)")
    elif snapshot["error"] and snapshot["state"] == "transitioning":
        level = WARN
    if snapshot.get("save_error"):
        level = max(level, WARN)
        messages.append(f"last map save incomplete: {snapshot['save_error']}")
    if snapshot["config_warnings"]:
        level = max(level, WARN)
        messages.extend(snapshot["config_warnings"])
    values = {"op.state": snapshot["state"], "op.target": snapshot["target"],
              "glim.pipeline": snapshot["pipeline"], "glim.session": snapshot["session"] or "-",
              "glim.restarts": snapshot["restarts"]}
    if uptime is not None:
        values["uptime_s"] = uptime
    if snapshot["last_session"]:
        values["glim.last_session"] = (f"{snapshot['last_session']['id']}: "
                                       f"{snapshot['last_session']['outcome']}")
    # GLIM's own failure goes in a glim.* value: health.error is reserved for failing to OBSERVE.
    if snapshot["error"]:
        values["glim.error"] = snapshot["error"]
    components.append(("service", level, "; ".join(messages), values, "current", {}))

    # odometry / imu ------------------------------------------------------------------------
    if demanded and not running and not starting:
        level, message, values, availability = (
            WARN, f"odometry: GLIM not running ({snapshot['detail']})", {}, "retrying")
    else:
        level, message, values, availability = _stream(
            "odometry", "odometry", odometry, now, demanded, starting, stall, limits,
            settings.get("expected_odometry_hz"))
    components.append(("odometry", level, message, values, availability, {}))
    # The IMU arrives from the sensor tier whether or not GLIM is up: always measured when demanded.
    components.append(("imu", *_stream("imu", f"IMU on {topics['imu']}", imu, now, demanded,
                                       starting, stall, limits), {}))

    # points -------------------------------------------------------------------------------
    if not demanded:
        components.append(("points", OK, f"points on {topics['points']}: standby (not demanded)",
                           {}, "paused", {}))
    elif points_publishers is None:
        components.append(("points", WARN, f"points on {topics['points']}: graph unavailable", {},
                           "unavailable", {"health.error": "ROS graph query failed"}))
    else:
        level = OK if points_publishers else WARN
        message = (f"points on {topics['points']}: {points_publishers} publisher(s)"
                   if points_publishers else f"no publisher on {topics['points']}")
        components.append(("points", level, message, {"publishers": points_publishers},
                           "current", {}))

    # storage ------------------------------------------------------------------------------
    if isinstance(storage, tuple):
        free_gb, free_pct = storage
        values = {"disk.free_gb": free_gb, "disk.free_pct": free_pct}
        messages = [f"{free_gb:.1f} GB free ({free_pct:.0f}%)"]
        level = apply_limits(OK, messages, values, limits.get("storage"))
        components.append(("storage", level, "; ".join(messages), values, "current", {}))
    else:
        components.append(("storage", WARN, "map storage unreadable", {}, "unavailable",
                           {"health.error": str(storage)}))

    statuses = []
    for component, level, message, values, availability, extra in components:
        meta = {
            "health.vehicle_id": identity.get("vehicle_id") or "",
            "health.instance": identity["instance"],
            "health.service": identity["service"],
            "health.publisher_id": publisher_id,
            "health.sequence": sequence,
            "health.publish_interval_s": settings["interval_s"],
            "health.sample_id": f"{publisher_id}-{sequence}",
            "health.sample_age_s": 0.0,
            "health.stale_after_s": settings["stale_after_s"],
            "health.availability": availability,
            **extra,
        }
        statuses.append({
            "level": level,
            "name": f"{identity['instance']}: {component}",
            "message": message,
            "hardware_id": "",
            "values": {key: _fmt(value) for key, value in {**values, **meta}.items()},
        })
    return statuses
