#!/usr/bin/env python3
"""Normalize Navtech's published 2022 ROS 2 road cloud export to ROS radar XYZI."""
import hashlib
import time

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


def is_road_cloud_export(bag_dir, metadata):
    # Other Navtech bags use the same topic and field names with different units.
    # Match the unmodified SQLite member of the manufacturer's ROS2 Bags.zip.
    files = metadata['relative_file_paths']
    if len(files) != 1 or metadata['storage_identifier'] != 'sqlite3':
        return False
    path = bag_dir / files[0]
    if path.stat().st_size != 457621504:
        return False
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest() == (
            '0bce78092533a0e432ed4321a99a4f77ffffb6eecc26244332379c7b99d741a7')


def normalize_road_cloud(msg, frame_id):
    """Convert packed clockwise bin coordinates from this export in place."""
    fields = [(f.name, f.offset, f.datatype, f.count) for f in msg.fields]
    expected = [(name, 4*i, PointField.FLOAT32, msg.width)
                for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
    if (fields != expected or msg.height != 1 or msg.point_step != 16
            or msg.row_step != 16*msg.width or len(msg.data) != msg.row_step):
        raise ValueError('unexpected layout for the Navtech road cloud export')

    points = np.frombuffer(msg.data, dtype='>f4' if msg.is_bigendian else '<f4').reshape(-1, 4)
    # The paired Colossus configuration gives 0.0438 metres/bin and zero offset.
    points[:, 0] *= 0.0438
    points[:, 1] *= -0.0438
    for field in msg.fields:
        field.count = 1
    msg.header.frame_id = frame_id
    return msg


def main():
    with rclpy.init():
        node = Node('navtech_road_cloud_adapter')
        try:
            frame = node.declare_parameter('frame_id', 'navtech',
                                           ParameterDescriptor(read_only=True)).value
            if not frame or frame.startswith('/'):
                raise ValueError('frame_id must be nonempty with no leading slash')
            output = node.create_publisher(PointCloud2, 'navtech_radar/point_cloud', 10)
            last_message = None

            def forward(msg):
                nonlocal last_message
                output.publish(normalize_road_cloud(msg, frame))
                last_message = time.monotonic()

            source = node.create_subscription(PointCloud2, '~/input', forward, 10)
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.25)
                # A paused rosbag player keeps its publisher. At EOF, drain any
                # queued data and let a headless launch exit after its last cloud.
                if (last_message is not None and time.monotonic() - last_message >= 0.25
                        and source.get_publisher_count() == 0):
                    output.wait_for_all_acked(Duration(seconds=2))
                    break
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            node.destroy_node()


if __name__ == '__main__':
    main()
