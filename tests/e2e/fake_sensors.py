"""Synthetic Ouster-like point clouds + SBG-like IMU for a GLIM smoke test (static vehicle).

/top/points: sensor_msgs/PointCloud2 (x, y, z, intensity float32; t uint32 ns since scan start)
at 10 Hz in frame ouster_lidar; /ins_main/imu/data: sensor_msgs/Imu at 200 Hz in frame imu_link,
at rest (specific force +g on z). The scene is a furnished 12 x 9 x 3 m room so scan matching has
structure in every direction.
"""
import random
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2, PointField

random.seed(7)


def room_points(count):
    pts = []
    boxes = [((2.0, 1.0, -1.2), (0.8, 0.8, 1.6)), ((-3.0, -2.0, -1.2), (1.5, 0.6, 1.0)),
             ((-1.0, 3.0, -1.2), (0.5, 2.0, 2.2))]
    for _ in range(count):
        r = random.random()
        if r < 0.6:   # walls
            side = random.randrange(4)
            if side == 0:
                x, y = 6.0, random.uniform(-4.5, 4.5)
            elif side == 1:
                x, y = -6.0, random.uniform(-4.5, 4.5)
            elif side == 2:
                x, y = random.uniform(-6, 6), 4.5
            else:
                x, y = random.uniform(-6, 6), -4.5
            z = random.uniform(-1.2, 1.8)
        elif r < 0.75:
            x, y, z = random.uniform(-6, 6), random.uniform(-4.5, 4.5), -1.2
        elif r < 0.85:
            x, y, z = random.uniform(-6, 6), random.uniform(-4.5, 4.5), 1.8
        else:
            (cx, cy, cz), (sx, sy, sz) = random.choice(boxes)
            face = random.randrange(5)
            u, v = random.random(), random.random()
            if face == 0:
                x, y, z = cx + sx * u, cy, cz + sz * v
            elif face == 1:
                x, y, z = cx + sx * u, cy + sy, cz + sz * v
            elif face == 2:
                x, y, z = cx, cy + sy * u, cz + sz * v
            elif face == 3:
                x, y, z = cx + sx, cy + sy * u, cz + sz * v
            else:
                x, y, z = cx + sx * u, cy + sy * v, cz + sz
        pts.append((x + random.gauss(0, 0.01), y + random.gauss(0, 0.01), z + random.gauss(0, 0.01)))
    return pts


class FakeSensors(Node):
    def __init__(self):
        super().__init__("fake_sensors")
        self.points = self.create_publisher(PointCloud2, "/top/points", qos_profile_sensor_data)
        self.imu = self.create_publisher(Imu, "/ins_main/imu/data", qos_profile_sensor_data)
        self.scene = room_points(60000)
        self.create_timer(0.1, self.publish_scan)
        self.create_timer(0.005, self.publish_imu)

    def publish_scan(self):
        stamp = self.get_clock().now()
        n = 20000
        sample = random.sample(self.scene, n)
        data = bytearray()
        for i, (x, y, z) in enumerate(sample):
            data += struct.pack("<ffffI", x, y, z, 100.0, int(i * 100_000_000 / n))
        msg = PointCloud2()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = "ouster_lidar"
        msg.height, msg.width = 1, n
        msg.fields = [PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                      PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                      PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                      PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
                      PointField(name="t", offset=16, datatype=PointField.UINT32, count=1)]
        msg.is_bigendian = False
        msg.point_step, msg.row_step = 20, 20 * n
        msg.is_dense = True
        msg.data = bytes(data)
        self.points.publish(msg)

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"
        msg.orientation.w = 1.0
        msg.linear_acceleration.x = random.gauss(0, 0.02)
        msg.linear_acceleration.y = random.gauss(0, 0.02)
        msg.linear_acceleration.z = 9.80665 + random.gauss(0, 0.02)
        msg.angular_velocity.x = random.gauss(0, 0.001)
        msg.angular_velocity.y = random.gauss(0, 0.001)
        msg.angular_velocity.z = random.gauss(0, 0.001)
        self.imu.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(FakeSensors())


if __name__ == "__main__":
    main()
