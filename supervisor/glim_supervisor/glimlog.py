"""GLIM's own complaints about its input, lifted from its log into fleet health.

GLIM validates every scan and IMU sample and says so — but only in its log, where nobody looks
until the map is wrong. The patterns below are GLIM's literal messages (glim/util/time_keeper.cpp,
glim/util/ros_cloud_converter.hpp, glim_ros2 glim_ros.cpp); each maps to a category with the
likely cause, so a day-one misconfiguration (unsynchronized clocks, a point type without per-point
time, IMU in g) surfaces on the dashboard within seconds.
"""
import re
import threading
import time

ANSI = re.compile(r"\x1b\[[0-9;]*m")

# (substring of GLIM's message, category). First match wins.
PATTERNS = (
    ("large time difference between points and imu", "time_sync"),
    ("IMU timestamp rewind detected", "imu_time"),
    ("skip an invalid IMU data", "imu_time"),
    ("large time gap between consecutive IMU data", "imu_gap"),
    ("unexpected acceleration norm", "imu_units"),
    ("large time gap between consecutive LiDAR frames", "lidar_gap"),
    ("skip an invalid point cloud", "lidar_time"),
    ("frame timestamp is negative", "lidar_time"),
    ("frame timestamp is wrong", "lidar_time"),
    ("failed to extract points from message", "point_format"),
    ("large point timestamp", "point_time"),
    ("negative per-point timestamp", "point_time"),
    ("large per-point timestamp", "point_time"),
    ("# of timestamps and # of points mismatch", "point_time"),
    ("inconsistent # of points and # of timestamps", "point_time"),
)

HINTS = {
    "time_sync": "lidar and IMU stamps are >1 s apart: put both on one clock (Ouster "
                 "timestamp_mode vs SBG time.source)",
    "imu_time": "IMU stamps go backwards: check the IMU's time source and for duplicate publishers",
    "imu_gap": "IMU samples arrive with >0.1 s gaps: check the IMU rate, QoS and transport",
    "imu_units": "IMU acceleration magnitude is neither ~9.8 m/s^2 nor ~1 g: check units/scale "
                 "(glim.ros.acc_scale)",
    "lidar_gap": "scans arrive with large gaps: check the lidar rate, QoS and transport",
    "lidar_time": "scan stamps are invalid or go backwards: check the lidar's timestamp_mode",
    "point_format": "point clouds cannot be decoded: check the point type and fields",
    "point_time": "per-point times are unusable: use a point type with a per-point `t` field "
                  "(Ouster point_type original)",
}


def classify(line):
    """-> category or None for one log line (ANSI colors stripped)."""
    text = ANSI.sub("", line)
    for needle, category in PATTERNS:
        if needle in text:
            return category
    return None


class InputWarnings:
    """Thread-safe per-session tally: count, last occurrence (monotonic) and last message."""

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.Lock()
        self.seen = {}

    def observe(self, line):
        category = classify(line)
        if category is None:
            return None
        with self.lock:
            entry = self.seen.setdefault(category, {"count": 0, "last": None, "message": ""})
            entry["count"] += 1
            entry["last"] = self.clock()
            entry["message"] = ANSI.sub("", line).strip()[-200:]
        return category

    def reset(self):
        with self.lock:
            self.seen = {}

    def snapshot(self):
        now = self.clock()
        with self.lock:
            return {category: {"count": entry["count"], "age_s": now - entry["last"],
                               "message": entry["message"], "hint": HINTS[category]}
                    for category, entry in self.seen.items()}
