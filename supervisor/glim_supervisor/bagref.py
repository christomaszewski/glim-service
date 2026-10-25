"""The evaluation reference for an offline remap, read from the same bags (rosbag2_py).

Live and replayed sessions record their reference from ROS as it arrives; glim_rosbag reads the
bags directly and publishes nothing, so a remap takes the reference topic from the bags itself.
"""


def write_reference(bags, topic, out_path, min_dt=0.02):
    """Write `topic` (nav_msgs/Odometry) from the bags as TUM; -> poses written."""
    import rosbag2_py
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message

    written, last = 0, None
    with open(out_path, "w") as out:
        for bag in bags:
            reader = rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(bag)), rosbag2_py.ConverterOptions("", ""))
            reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
            while reader.has_next():
                _, data, _ = reader.read_next()
                message = deserialize_message(data, Odometry)
                stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
                if last is not None and stamp - last < min_dt * 0.9:   # keeps a 50 Hz source whole
                    continue
                last = stamp
                p, q = message.pose.pose.position, message.pose.pose.orientation
                out.write(f"{stamp:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} "
                          f"{q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n")
                written += 1
    if not written:
        raise ValueError(f"no {topic} messages in the bags")
    return written
