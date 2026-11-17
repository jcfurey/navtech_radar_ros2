#!/usr/bin/env python3
"""Check the rendered radar bearing against vehicle odometry during the TAMU route 1/run 2 turn."""
import argparse
from contextlib import ExitStack
import math
from pathlib import Path

from nav_msgs.msg import Odometry
from nav_ross.msg import HighPrecisionFFTData, nav_msg
from navtech_radar_msgs.msg import RadarConfiguration
import numpy as np
import rclpy
from rclpy.serialization import deserialize_message
import rosbag2_py
import yaml

from compare_images import ImageComparison, make_sweep


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def turn_sweeps(bag):
    info = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    start = info['starting_time']['nanoseconds_since_epoch']
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/Navtech/Configuration_Topic']))
    _, data, *_ = reader.read_next_ext()
    legacy = deserialize_message(data, nav_msg)
    config = RadarConfiguration(configuration_version=1, range_resolution=legacy.range_resolution,
                                range_gain=1.0, range_bins=legacy.RangeInBins,
                                encoder_size=legacy.EncoderSize, azimuth_samples=legacy.AzimuthSamples,
                                expected_rotation_rate=legacy.ExpectedRotationRate, sample_bit_depth=16)
    config.header.frame_id = 'lester/navtech'
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/Navtech/FFTData', '/lester/odom']))
    reader.seek(start + 868_700_000_000)
    poses, group, sweeps = [], [], []
    while reader.has_next():
        topic, raw, stamp, *_ = reader.read_next_ext()
        if stamp > start + 871_000_000_000:
            break
        if topic == '/lester/odom':
            msg = deserialize_message(raw, Odometry)
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            poses.append((seconds(msg.header.stamp), p.x, p.y, yaw))
        else:
            spoke = deserialize_message(raw, HighPrecisionFFTData)
            if group and group[-1].azimuth > spoke.azimuth:
                if len(group) == len({s.azimuth for s in group}) == config.azimuth_samples:
                    sweeps.append(make_sweep(config, group, len(sweeps)))
                group = []
            group.append(spoke)
    if len(sweeps) < 5 or len(poses) < 2:
        raise ValueError('Expected five complete sweeps and odometry near 869 s in TAMU route 1/run 2')
    pair = sweeps[0], sweeps[4]
    midpoints = [(seconds(s.header.stamp) + seconds(s.end_stamp)) / 2 for s in pair]
    odom = np.asarray(poses)
    assert odom[0, 0] <= midpoints[0] < midpoints[1] <= odom[-1, 0]
    assert np.all(np.diff(odom[:, 0]) >= 0)
    odom[:, 3] = np.unwrap(odom[:, 3])
    dx, dy, dyaw = [np.diff(np.interp(midpoints, odom[:, 0], channel)).item()
                     for channel in odom[:, 1:].T]
    return config, pair, math.hypot(dx, dy), math.degrees(dyaw)


def sample_bearings(image, count):
    # Read the plan view using its ROS axes (+x up, +y left). Sample radii
    # 12..45 m, well away from the centre and edge of the 64 m view.
    center = (image.width - 1) / 2
    pixels = np.linspace(12, 45, 133) * (center / 64)
    angles = np.arange(count)[:, None] * (2 * np.pi / count)
    rows = np.rint(center - np.cos(angles) * pixels).astype(int)
    cols = np.rint(center - np.sin(angles) * pixels).astype(int)
    bgr = np.frombuffer(image.data, np.uint8).reshape(image.height, image.width, 3)
    levels = bgr[rows, cols].mean(axis=2)
    return (levels - levels.mean(axis=0)) / np.maximum(1.0, levels.std(axis=0))


def main(bag):
    config, pair, distance, yaw = turn_sweeps(bag)
    parameters = ['annotate:=false', 'max_range:=64.0', 'cartesian_size:=513', 'contrast_alpha:=1.0']
    with rclpy.init(), ExitStack() as cleanup:
        check = ImageComparison(cleanup, parameters, views=('cartesian',))
        check.configs.publish(config)
        grids = [sample_bearings(check.frame(s, warmup=(i == 0))['cartesian'], config.azimuth_samples)
                 for i, s in enumerate(pair)]
    first, last = (np.fft.rfft(g, axis=0) for g in grids)
    count = config.azimuth_samples
    correlation = np.fft.irfft((first.conj() * last).sum(axis=1), n=count) / grids[0].size
    peak = int(np.argmax(correlation))
    shift = ((peak + count // 2) % count - count // 2) * 360 / count
    print(f'Translation {distance:.3f} m, vehicle yaw change {yaw:+.3f} deg, '
          f'scene bearing change {shift:+.3f} deg, correlation {correlation[peak]:.3f}.')
    assert distance < 0.1 and yaw < -15, 'Expected the nearly stationary clockwise turn'
    assert correlation[peak] > 0.4 and abs(shift + yaw) < 2.0, 'Scene must rotate opposite vehicle yaw'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    main(parser.parse_args().bag)
