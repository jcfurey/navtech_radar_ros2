#!/usr/bin/env python3
"""Compare the 2022 Navtech ROS 2 road cloud export with its original COLRAW capture."""
import argparse
from contextlib import closing
from pathlib import Path

from navtech_recording.formats import colraw_rotations
from navtech_recording.road_cloud import is_road_cloud_export, normalize_road_cloud
import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
import yaml


def road_messages(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/radar_data/point_cloud']))
    while reader.has_next():
        _, payload, *_ = reader.read_next_ext()
        yield deserialize_message(payload, PointCloud2)


def check(bag, colraw):
    meta = yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    if not is_road_cloud_export(bag, meta):
        raise ValueError('expected the original ThresholdedPointCloud bag from ROS2 Bags.zip')
    messages = points = origins = 0
    maximum_error = 0.
    with closing(colraw_rotations(colraw)) as frames:
        prev = next(frames)
        for msg, following in zip(road_messages(bag), frames, strict=True):
            # This exporter closes a turn after appending the ray crossing zero.
            stamp = msg.header.stamp
            assert stamp.sec*10**9 + stamp.nanosec == following.sensor_ns[0]
            geom = prev.geometry
            assert geom == following.geometry and geom.rows == 400 and geom.encoder == 5600
            assert geom.spacing == 0.0438 and geom.gain == 1 and geom.offset == 0
            start = 0 if messages == 0 else 1
            data = np.vstack((prev.data[start:], following.data[:1]))
            encoders = np.r_[prev.encoders[start:], following.encoders[:1]]
            source = np.full((geom.rows, geom.bins), -1, dtype=np.int16)
            source[encoders.astype(int)*geom.rows//geom.encoder] = data

            # Decode the four physical words directly. The original PointField.count
            # values would make a standard consumer read past the end of a point.
            packed = np.frombuffer(msg.data, '<f4').reshape(-1, 4).astype(float)
            radius = np.rint(np.hypot(packed[:, 0], packed[:, 1])).astype(int)
            angle = np.arctan2(packed[:, 1], packed[:, 0])
            cell = np.rint(angle*geom.rows/(2*np.pi)).astype(int) % geom.rows
            assert np.all((radius >= 0) & (radius < geom.bins))
            nonzero = radius > 0
            np.testing.assert_array_equal(source[cell[nonzero], radius[nonzero]], packed[nonzero, 3])

            result = normalize_road_cloud(msg, 'navtech')
            decoded = read_points(result, skip_nans=False)
            assert len(decoded) == len(radius) and np.all(decoded['z'] == 0)
            np.testing.assert_array_equal(decoded['intensity'], packed[:, 3])
            theta = -cell * (2*np.pi/geom.rows)
            metres = radius*geom.spacing
            error = np.hypot(decoded['x'] - metres*np.cos(theta), decoded['y'] - metres*np.sin(theta))
            maximum_error = max(maximum_error, error.max(initial=0.))
            origins += int((~nonzero).sum())
            points += msg.width
            messages += 1
            prev = following
    assert messages == meta['message_count'] and points > origins and maximum_error < 2e-5
    print(f'{messages:,} clouds, {points:,} points, {points-origins:,} nonzero-range returns '
          f'matched to source intensity, {origins:,} at the origin. '
          f'Maximum horizontal conversion residual {maximum_error:.8f} m.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path, help='extracted ROS 2 thresholded point-cloud bag directory')
    parser.add_argument('colraw', type=Path, help='original moving-road-vehicle Colossus file')
    args = parser.parse_args()
    check(args.bag, args.colraw)
