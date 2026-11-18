#!/usr/bin/env python3
"""Stream public Navtech radar recordings in recorded time."""
import argparse
from array import array
from contextlib import closing
import math
from pathlib import Path
import time

from builtin_interfaces.msg import Time
from navtech_radar_msgs.msg import RecordedRotation
from navtech_recording.formats import colraw_rotations, dataset_rotations, identify, radiate_cartesian_images
from navtech_recording import ros1
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from rosgraph_msgs.msg import Clock
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Image


def stamp(ns):
    sec, nano = divmod(int(ns), 10**9)
    return Time(sec=sec, nanosec=nano)


def packed(code, values, dtype):
    result = array(code)
    result.frombytes(np.asarray(values, dtype=dtype).tobytes())
    return result


def make_rotation(frame, profile, frame_id, version):
    msg = RecordedRotation()
    geom = frame.geometry
    cfg = msg.configuration
    img = msg.image
    img.header.frame_id = frame_id
    img.header.stamp = stamp(frame.stamp)
    cfg.header = img.header
    cfg.configuration_version = version
    cfg.azimuth_samples = geom.rows
    cfg.encoder_size = geom.encoder
    cfg.range_bins = geom.bins
    cfg.bin_size = round(geom.spacing*10000)
    cfg.range_resolution = geom.spacing
    cfg.range_gain = geom.gain
    cfg.range_offset = geom.offset
    cfg.expected_rotation_rate = geom.hz
    cfg.packet_rate = geom.packet_rate
    cfg.sample_bit_depth = frame.data.dtype.itemsize*8
    img.height, img.width = frame.data.shape
    img.step = img.width * frame.data.dtype.itemsize
    img.encoding = 'mono8' if cfg.sample_bit_depth == 8 else 'mono16'
    img.is_bigendian = frame.data.dtype.byteorder == '>'
    img.data = array('B', frame.data.tobytes(order='C'))
    msg.encoder_values = packed('H', frame.encoders, np.uint16)
    msg.sweep_counters = packed('H', frame.counters, np.uint16)
    msg.sensor_ns = packed('Q', frame.sensor_ns, np.uint64)
    msg.receipt_ns = packed('Q', frame.receipt_ns, np.uint64)
    msg.row_flags = packed('B', frame.flags, np.uint8)
    msg.source_format = profile
    return msg


def events(node, path, profile, args):
    if profile == 'radiate_cartesian':
        pub = node.create_publisher(Image, 'navtech_radar/fft/cartesian', 4)
        for ns, pixels in radiate_cartesian_images(path):
            msg = Image()
            msg.header.frame_id = args.frame_id
            msg.header.stamp = stamp(ns)
            msg.height, msg.width = pixels.shape
            msg.encoding = 'mono8'
            msg.step = msg.width
            msg.data = array('B', pixels.tobytes())
            yield ns, pub, msg
        return
    if profile == 'ros1':
        publishers = {k: node.create_publisher(get_message(typ), name, 10)
                      for k, (name, typ) in ros1.TOPICS.items()}
        with closing(ros1.messages(path)) as stream:
            for ns, topic, data in stream:
                yield ns, publishers[topic], data
        return
    if profile == 'cpr':
        sidecar = path.with_suffix('.colraw')
        if not sidecar.is_file():
            raise ValueError(f'convert CPR with Navtech’s converter and place the result at {sidecar}')
        rotations = colraw_rotations(sidecar, cpr=path)
    elif profile == 'colraw':
        rotations = colraw_rotations(path)
    else:
        rotations = dataset_rotations(path, profile)
    pub = node.create_publisher(RecordedRotation, 'navtech_radar/recording/rotation', 4)
    if args.wait_for_assembler:
        deadline = time.monotonic() + 30
        while pub.get_subscription_count() == 0:
            if not rclpy.ok():
                return
            if time.monotonic() >= deadline:
                raise RuntimeError('sweep assembler did not subscribe to recorded rotations')
            time.sleep(0.05)
    version, previous = 0, None
    with closing(rotations):
        for frame in rotations:
            key = (frame.geometry, frame.data.dtype)
            if key != previous:
                version += 1
                previous = key
            yield frame.stamp, pub, make_rotation(frame, profile, args.frame_id, version)


def play(node, args):
    path = args.recording.expanduser()
    profile = identify(path, args.format)
    if profile == 'ros2':
        raise ValueError('use playback.launch.py with bag:= for a ROS 2 bag')
    clock = node.create_publisher(Clock, '/clock', 10) if args.clock else None
    node.get_logger().info(f'Opening {path} ({profile})')
    count, previous = 0, None
    active_publishers = set()
    with closing(events(node, path, profile, args)) as stream:
        for ns, pub, data in stream:
            if previous is None:
                # Give late-joining stages time to discover publishers after archive I/O.
                deadline = time.monotonic() + args.delay
            else:
                gap = max(0, (ns-previous)*1e-9)
                deadline += (min(gap, args.max_gap) if args.max_gap else gap)/args.rate
            while rclpy.ok() and (remaining := deadline-time.monotonic()) > 0:
                time.sleep(min(remaining, 0.05))
            if not rclpy.ok():
                return
            if clock:
                clock.publish(Clock(clock=stamp(ns)))
            pub.publish(data)
            active_publishers.add(pub)
            count += 1
            previous = ns
            if args.limit and count >= args.limit:
                break
    if not count:
        raise ValueError(f'no radar measurements found in {path} using format {profile}')
    node.get_logger().info(f'Finished {count} recorded frames/messages')
    for publisher in active_publishers:
        publisher.wait_for_all_acked(rclpy.duration.Duration(seconds=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--format', default='auto', choices=['auto', 'colraw', 'cpr', 'ros1',
                        'oxford', 'oord', 'radiate', 'radiate_cartesian', 'boreas', 'boreas_rt', 'mulran', 'mulran_ray', 'muses'])
    parser.add_argument('--frame-id', default='navtech')
    parser.add_argument('--rate', type=float, default=1.0)
    parser.add_argument('--max-gap', type=float, default=0.0)
    parser.add_argument('--delay', type=float, default=2.0)
    parser.add_argument('--limit', type=int, default=0)
    parser.add_argument('--clock', action='store_true')
    parser.add_argument('--wait-for-assembler', action='store_true')
    args = parser.parse_args(remove_ros_args()[1:])
    if (not args.frame_id or args.frame_id.startswith('/') or args.limit < 0 or args.rate <= 0
            or any(not math.isfinite(v) or v < 0 for v in (args.rate, args.max_gap, args.delay))):
        parser.error('invalid frame, rate, delay, gap or limit')
    rclpy.init()
    node = Node('navtech_recording_player')
    try:
        play(node, args)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
