"""Wait until a GLIM instance is really mapping: odometry at rate, health saying so.

  python3 probe.py <namespace> <min odometry Hz> <timeout s>

Succeeds when GLIM's <ns>/glim_ros/odom arrives at >= the given rate AND one /diagnostics
publication reports the service active with an OK odometry component that measured a rate.
Exits 1 with what was last seen otherwise. Runs inside the GLIM image (rclpy).
"""
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry


def main():
    namespace, min_rate, timeout = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
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
    deadline = time.monotonic() + timeout
    rate = 0.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        recent = [t for t in arrivals if t > time.monotonic() - 3.0]
        rate = (len(recent) - 1) / (recent[-1] - recent[0]) if len(recent) > 2 else 0.0
        if (rate >= min_rate and health.get("state") == "active"
                and health.get("odometry_level") == 0 and "Hz" in health.get("odometry", "")):
            print(f"probe: odometry {rate:.1f} Hz; health: {health['service']} | {health['odometry']}")
            return 0
    print(f"probe: FAILED after {timeout:.0f}s — odometry {rate:.1f} Hz "
          f"({len(arrivals)} messages); last health: {health or 'none'}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
