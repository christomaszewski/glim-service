#!/usr/bin/env python3
"""Stand-in for glim_rosnode that honors the contract the supervisor relies on.

  argv     --ros-args -r __ns:=<ns> -p config_path:=<dir> -p dump_path:=<dir> -p use_sim_time:=<b>
  SIGINT   finish and serialize the map into dump_path, exit 0 (GLIM's spin() -> wait() -> save())

The behavior of each run comes from the file named by FAKE_GLIM_CONTROL (read once at startup, so
a test can change it between sessions): save (default) | crash | hang | fail_save | slow_save.
With FAKE_GLIM_ROS=1 it also runs an rclpy node `glim_ros` in <ns> publishing ~/odom at 10 Hz.
"""
import json
import os
from pathlib import Path
import signal
import sys
import time

args = sys.argv[1:]
params = {}
for flag, value in zip(args, args[1:]):
    if flag in ("-p", "-r") and ":=" in value:
        key, val = value.split(":=", 1)
        params[key] = val
control = os.environ.get("FAKE_GLIM_CONTROL")
mode = Path(control).read_text().strip() if control and Path(control).exists() else "save"
config_dir, dump = Path(params["config_path"]), Path(params["dump_path"])
if not (config_dir / "config.json").is_file():
    sys.exit(f"fake glim: no config.json in {config_dir}")   # GLIM aborts the same way
(config_dir.parent / "argv.json").write_text(json.dumps(args))

stop = []
signal.signal(signal.SIGINT, lambda *_: stop.append(1))
signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
if mode == "hang":
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
if mode == "crash":
    time.sleep(0.2)
    sys.exit(3)

node = None
if os.environ.get("FAKE_GLIM_ROS") == "1":
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from nav_msgs.msg import Odometry
    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("glim_ros", namespace=params["__ns"])
    publisher = node.create_publisher(Odometry, "~/odom", 10)

    def tick():
        message = Odometry()
        message.header.stamp = node.get_clock().now().to_msg()
        message.header.frame_id = "odom"
        publisher.publish(message)
    node.create_timer(0.1, tick)

while not stop:
    if node is not None:
        rclpy.spin_once(node, timeout_sec=0.05)
    else:
        time.sleep(0.05)
if node is not None:
    node.destroy_node()
    rclpy.shutdown()
if mode == "fail_save":
    sys.exit(1)
if mode == "slow_save":
    time.sleep(2)
dump.mkdir(parents=True, exist_ok=True)
(dump / "graph.txt").write_text("fake map\n")
sys.exit(0)
