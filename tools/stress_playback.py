#!/usr/bin/env python3
"""Pace recorded or synthetic spokes through all three installed C++ nodes, with six active outputs."""
import argparse
from array import array
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
from copy import deepcopy
import hashlib
import math
from pathlib import Path
import subprocess
import time
from uuid import uuid4

from ament_index_python.packages import get_package_prefix
from nav_ross.msg import HighPrecisionFFTData, nav_msg
from navtech_radar_msgs.msg import FftSweep, RadarConfiguration
import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image, LaserScan, PointCloud2

from compare_playback import cpu_seconds, recorded_input, stop


def stamp(header):
    return header.stamp.sec * 10**9 + header.stamp.nanosec


def synthetic_input(rotations, scene):
    cfg = nav_msg(range_resolution=0.043800000101327896, AzimuthSamples=400, EncoderSize=5600,
                  BinSize=438, RangeInBins=6000, ExpectedRotationRate=4)
    rng = np.random.default_rng(350)
    templates = []
    for _ in range(cfg.AzimuthSamples):
        row = array('H')
        row.frombytes(rng.integers(0, 65536 if scene == 'dense' else 8192,
                                   cfg.RangeInBins, dtype=np.uint16).tobytes())
        templates.append(row)

    def packet(i):
        turn, row = divmod(i, cfg.AzimuthSamples)
        ray = HighPrecisionFFTData(angle=row * 360. / cfg.AzimuthSamples, azimuth=14 * row,
                                   size=cfg.RangeInBins, sweepCounter=i % 65536,
                                   data=templates[(turn + row) % cfg.AzimuthSamples])
        sec, nsec = divmod(i * 625_000, 10**9)
        ray.header.stamp.sec = 10 + sec
        ray.header.stamp.nanosec = nsec
        return ray

    count = cfg.AzimuthSamples
    groups = [[packet(turn * count + row) for row in range(count)] for turn in range(rotations)]
    return cfg, groups, packet(rotations * count)


def run(bag, rotations, hz, composed, scene):
    if scene:
        cfg, groups, sentinel = synthetic_input(rotations, scene)
    else:
        cfg, groups, sentinel = recorded_input(bag, rotations)
        groups = groups[1:]  # The first recorded revolution may be partial.
    count = cfg.AzimuthSamples
    if any(len(rows) != count for rows in groups):
        raise ValueError('Choose a segment with full revolutions for the paced test')
    stamps = [stamp(rows[0].header) for rows in groups]
    expected = {}
    packets = []
    for key, rows in zip(stamps, groups):
        digest = hashlib.sha256()
        for ray in sorted(rows, key=lambda s: s.azimuth):
            digest.update(np.asarray(ray.data, dtype='<u2'))
        expected[key] = digest.digest()
        packets.extend(serialize_message(ray) for ray in rows)
    packets.append(serialize_message(sentinel))
    # A two-row probe after the test interval lets the first measured packet
    # rewind the nodes and clear all state left over from discovery.
    low = deepcopy(sentinel)
    low.header.stamp.sec += 1
    low.angle, low.azimuth = 0., 0
    high = deepcopy(low)
    high.azimuth = round(0.75 * cfg.EncoderSize)
    high.angle = high.azimuth * 360. / cfg.EncoderSize
    del groups

    topics = {'fft/sweep': FftSweep, 'fft/image': Image, 'scan': LaserScan,
              'point_cloud': PointCloud2, 'fft/display': Image, 'fft/cartesian': Image}
    received = {topic: {} for topic in topics}
    probe, bad, unexpected = set(), set(), set()
    ns = '/navtech_load_' + uuid4().hex
    with rclpy.init(), ExitStack() as cleanup:
        node = rclpy.create_node('load_test', namespace=ns)
        cleanup.callback(node.destroy_node)
        configs = node.create_publisher(nav_msg, 'legacy_config', 1)
        spokes = node.create_publisher(HighPrecisionFFTData, 'legacy_fft', 512)
        configured = []
        node.create_subscription(RadarConfiguration, 'navtech_radar/configuration', configured.append,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        def collect(topic, msg):
            key = stamp(msg.header)
            if key == stamp(low.header):
                probe.add(topic)
                return
            if key not in expected:
                unexpected.add((topic, key))
                return
            if key in received[topic]:
                bad.add((topic, key))
            received[topic][key] = time.monotonic()
            if topic in ('fft/image', 'fft/sweep'):
                image = msg.image if topic == 'fft/sweep' else msg
                if (image.height != count or image.width != cfg.RangeInBins or
                        hashlib.sha256(image.data).digest() != expected[key]):
                    bad.add((topic, key))
            elif scene and topic == 'scan':
                valid = np.isfinite(msg.ranges) if scene == 'dense' else np.isnan(msg.ranges)
                if len(valid) != count or not np.all(valid):
                    bad.add((topic, key))
            elif scene and topic == 'point_cloud':
                # This seeded dense fixture saturates the 48-return budget
                # on every spoke. All quiet samples are below threshold.
                if msg.width != (count * 48 if scene == 'dense' else 0):
                    bad.add((topic, key))

        for topic, kind in topics.items():
            qos = qos_profile_sensor_data if topic in ('scan', 'point_cloud') else 8
            node.create_subscription(kind, 'navtech_radar/' + topic,
                                     lambda msg, topic=topic: collect(topic, msg), qos)

        procs = {}
        if composed:
            pkg = 'rclcpp_components'
            container = Path(get_package_prefix(pkg)) / 'lib' / pkg / 'component_container'
            process = subprocess.Popen([str(container), '--executor-type', 'multi-threaded', '--ros-args',
                                        '-r', '__node:=container', '-r', '__ns:=' + ns, '-p', 'thread_num:=3'])
            cleanup.callback(stop, process)
            procs['container'] = process
        for pkg, exe, plugin, params in [
            ('navtech_radar_driver', 'legacy_bag_driver', 'LegacyBagDriver',
             ['legacy_configuration_topic:=legacy_config', 'legacy_fft_topic:=legacy_fft']),
            ('navtech_radar_processing', 'fft_sweep_assembler', 'radar::SweepAssembler',
             ['publish_scan:=true', 'publish_point_cloud:=true', 'peak_selection:=cfar']),
            ('navtech_radar_processing', 'fft_sweep_visualizer', 'radar::SweepVisualizer', []),
        ]:
            if composed:
                args = ['ros2', 'component', 'load', ns + '/container', pkg, plugin, '--no-daemon',
                        '--node-namespace', ns, '--timeout', '15', '-e', 'use_intra_process_comms:=true']
            else:
                binary = Path(get_package_prefix(pkg)) / 'lib' / pkg / exe
                args = [str(binary), '--ros-args', '-r', '__ns:=' + ns]
            for param in params:
                args.extend(['-p', param])
            if composed:
                subprocess.run(args, check=True, timeout=25)
            else:
                procs[exe] = subprocess.Popen(args)
                cleanup.callback(stop, procs[exe])

        deadline = time.monotonic() + 15
        retry = 0
        while probe != topics.keys():
            now = time.monotonic()
            if now > deadline or any(p.poll() is not None for p in procs.values()):
                raise RuntimeError(f'Failed to warm up {topics.keys() - probe}')
            if now >= retry:
                configs.publish(cfg)
                if configured:
                    for packet in (low, high, low):
                        spokes.publish(packet)
                retry = now + 0.25
            rclpy.spin_once(node, timeout_sec=0.01)

        boundary = {}

        def feed():
            start = time.monotonic()
            # Batches of eight avoid relying on sub-millisecond sleep precision.
            for i in range(0, len(packets), 8):
                time.sleep(max(0., start + i / (count * hz) - time.monotonic()))
                for j in range(i, min(i + 8, len(packets))):
                    if j and j % count == 0:
                        boundary[stamps[j // count - 1]] = time.monotonic()
                    spokes.publish(packets[j])
            return time.monotonic() - start

        cpu_start = {name: cpu_seconds(p) for name, p in procs.items()}
        with ThreadPoolExecutor(max_workers=1) as sender:
            future = sender.submit(feed)
            while not future.done():
                rclpy.spin_once(node, timeout_sec=0.01)
            elapsed = future.result()
        deadline = time.monotonic() + 2
        while any(len(r) < rotations for r in received.values()) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        complete = set.intersection(*(set(r) for r in received.values()))
        complete.difference_update(key for _, key in bad)
        offered = rotations / elapsed
        print(f'{rotations} revolutions at {hz:g} Hz requested, {offered:.2f} Hz offered')
        print('Delivered: ' + ', '.join(f'{topic} {len(r)}/{rotations}' for topic, r in received.items()))
        print(f'{len(complete)} complete; {len(bad)} incomplete, mismatched or duplicate products; '
              f'{len(unexpected)} products with unexpected start stamps')
        if len(complete) < rotations:
            affected = [i + 1 for i, key in enumerate(stamps) if key not in complete]
            print('First affected turns (1-based):', affected[:10])
        if complete:
            latency = [1000 * (max(r[key] for r in received.values()) - boundary[key]) for key in complete]
            print('Last required product after encoder wrap, p50/p95/max (ms):',
                  ' / '.join(f'{v:.1f}' for v in np.percentile(latency, [50, 95, 100])))
        print('Node CPU (s): ' + ', '.join(f'{name} {cpu_seconds(p)-cpu_start[name]:.2f}'
                                           for name, p in procs.items()))
        for name, process in procs.items():
            with Path(f'/proc/{process.pid}/status').open() as status:
                peak_kib = next(int(line.split()[1]) for line in status if line.startswith('VmHWM:'))
            print(f'{name} peak RSS: {peak_kib / 1024:.1f} MiB')
        if offered < 0.95 * hz:
            print('Feeder delivered below 95% of the requested rate.')
            return False
        return len(complete) == rotations and not unexpected


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('bag', nargs='?')
    source.add_argument('--synthetic', choices=['quiet', 'dense'], help='Seeded 400 × 6000 FFT fixture')
    parser.add_argument('--rotations', type=int, default=32)
    parser.add_argument('--hz', type=float, default=4.0, help='Offered revolutions per wall second')
    parser.add_argument('--composed', action='store_true', help='Use C++ intra-process delivery')
    args = parser.parse_args()
    if args.rotations < 1 or not math.isfinite(args.hz) or args.hz <= 0:
        parser.error('rotations and hz must be positive')
    raise SystemExit(0 if run(args.bag, args.rotations, args.hz, args.composed, args.synthetic) else 1)
