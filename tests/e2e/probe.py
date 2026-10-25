"""Wait until a GLIM instance is really mapping: odometry at rate, health saying so.

  python3 probe.py <namespace> <min odometry Hz> <timeout s> [tf: expect | forbid]

Succeeds when GLIM's <ns>/glim_ros/odom arrives at >= the given rate AND one /diagnostics
publication reports the service active with an OK odometry component that measured a rate.
With `expect`/`forbid` it also requires GLIM's map -> odom transform on /tf to be seen / never
seen. Exits 1 with what was last seen otherwise. Runs inside the GLIM image (rclpy).
"""
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage


def main():
    namespace, min_rate, timeout = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
    tf_mode = sys.argv[4] if len(sys.argv) > 4 else None
    rclpy.init()
    node = rclpy.create_node("glim_e2e_probe")
    arrivals = []
    health = {}

    def on_diagnostics(message):
        statuses = {s.name: s for s in message.status}
        service = statuses.get(f"{namespace}: service")
        odometry = statuses.get(f"{namespace}: odometry")
        if service and odometry:
            health.update(service=service.message, odometry=odometry.message,
                          odometry_level=odometry.level[0] if isinstance(odometry.level, bytes)
                          else odometry.level,
                          state={kv.key: kv.value for kv in service.values}.get("op.state"))

    node.create_subscription(Odometry, f"/{namespace}/glim_ros/odom",
                             lambda _: arrivals.append(time.monotonic()), 10)
    node.create_subscription(DiagnosticArray, "/diagnostics", on_diagnostics, 10)
    tf_seen = []
    node.create_subscription(TFMessage, "/tf", lambda m: tf_seen.extend(
        (t.header.frame_id, t.child_frame_id) for t in m.transforms
        if (t.header.frame_id, t.child_frame_id) == ("map", "odom")), 100)
    deadline = time.monotonic() + timeout
    rate = 0.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        recent = [t for t in arrivals if t > time.monotonic() - 3.0]
        span = recent[-1] - recent[0] if len(recent) > 2 else 0.0
        rate = (len(recent) - 1) / span if span >= 2.0 else 0.0   # a queued burst is not a rate
        if tf_mode == "forbid" and tf_seen:
            print("probe: FAILED — GLIM published map -> odom with publish_tf off", file=sys.stderr)
            return 1
        if (rate >= min_rate and health.get("state") == "active"
                and health.get("odometry_level") == 0 and "Hz" in health.get("odometry", "")
                and (tf_mode != "expect" or tf_seen)
                and (tf_mode != "forbid" or arrivals[-1] - arrivals[0] >= 5.0)):
            print(f"probe: odometry {rate:.1f} Hz; health: {health['service']} | {health['odometry']}"
                  + (f"; tf map->odom seen {len(tf_seen)}x" if tf_mode == "expect" else "")
                  + ("; no TF from GLIM" if tf_mode == "forbid" else ""))
            return 0
    print(f"probe: FAILED after {timeout:.0f}s — odometry {rate:.1f} Hz "
          f"({len(arrivals)} messages); last health: {health or 'none'}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
