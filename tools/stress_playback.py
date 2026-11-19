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
import tempfile
import time
from uuid import uuid4

from ament_index_python.packages import get_package_prefix
from diagnostic_msgs.msg import DiagnosticArray
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


def write_paced_bag(path, namespace, config, packets, hz):
    import rosbag2_py as bag
    writer = bag.SequentialWriter()
    writer.open(bag.StorageOptions(uri=str(path), storage_id='mcap'), bag.ConverterOptions('cdr', 'cdr'))
    config_topic, fft_topic = namespace + '/legacy_config', namespace + '/legacy_fft'
    writer.create_topic(bag.TopicMetadata(0, config_topic, 'nav_ross/msg/nav_msg', 'cdr'))
    writer.create_topic(bag.TopicMetadata(1, fft_topic, 'nav_ross/msg/HighPrecisionFFTData', 'cdr'))
    writer.write(config_topic, serialize_message(config), 0)
    for i, packet in enumerate(packets):
        writer.write(fft_topic, packet, 1_000_000_000 + round(i * 1e9 / (hz * config.AzimuthSamples)))
    writer.close()


def run(bag, rotations, hz, composed, scene, native_player=False):
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
        if native_player:
            directory = cleanup.enter_context(tempfile.TemporaryDirectory(prefix='navtech-stress-'))
            mcap = Path(directory) / 'input'
            write_paced_bag(mcap, ns, cfg, packets, hz)
        node = rclpy.create_node('load_test', namespace=ns)
        cleanup.callback(node.destroy_node)
        configs = node.create_publisher(nav_msg, 'legacy_config', 1)
        spokes = node.create_publisher(HighPrecisionFFTData, 'legacy_fft', 512)
        configured = []
        reports = {}

        def diagnostic(msg):
            for status in msg.status:
                values = {v.key: v.value for v in status.values}
                if 'received_spokes' in values:
                    reports['assembler'] = (time.monotonic(), values)
                elif 'invalid_spokes' in values:
                    reports['adapter'] = (time.monotonic(), values)

        node.create_subscription(DiagnosticArray, 'diagnostics', diagnostic, 10)

        def snapshot():
            start = time.monotonic()
            while len(reports) < 2 or any(t < start for t, _ in reports.values()):
                if time.monotonic() - start > 4:
                    raise RuntimeError('Timed out waiting for adapter and assembler diagnostics')
                rclpy.spin_once(node, timeout_sec=0.05)
            return {stage: values for stage, (_, values) in reports.items()}

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
            process = subprocess.Popen([str(container), '--isolated', '--executor-type', 'events-cbg', '--ros-args',
                                        '-r', '__node:=container', '-r', '__ns:=' + ns, '-p', 'thread_num:=1'])
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

        initial = snapshot()
        cpu_start = {name: cpu_seconds(p) for name, p in procs.items()}
        if native_player:
            binary = Path(get_package_prefix('navtech_radar_driver')) / 'lib/navtech_radar_driver/bag_player'
            player = subprocess.Popen([str(binary), str(mcap)])
            cleanup.callback(stop, player)
            deadline = time.monotonic() + rotations / hz + 30
            while player.poll() is None:
                if time.monotonic() > deadline:
                    raise RuntimeError('Native bag playback timed out')
                rclpy.spin_once(node, timeout_sec=0.01)
            if player.returncode:
                raise RuntimeError(f'Native bag player exited with status {player.returncode}')
        else:
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
        if native_player:
            # Measure delivery of all six products between the first and last
            # complete turns. The player and observer use independent clocks.
            indices = [i for i, key in enumerate(stamps) if key in complete]
            measured_rate = None
            if len(indices) > 1:
                first, last = indices[0], indices[-1]
                first_time = max(r[stamps[first]] for r in received.values())
                last_time = max(r[stamps[last]] for r in received.values())
                measured_rate = (last - first) / (last_time - first_time)
            print(f'{rotations} revolutions scheduled at {hz:g} Hz by the C++ bag player')
            if measured_rate is not None:
                print(f'All-output delivery rate: {measured_rate:.2f} Hz')
        else:
            measured_rate = rotations / elapsed
            print(f'{rotations} revolutions at {hz:g} Hz requested, {measured_rate:.2f} Hz offered')
        print('Delivered: ' + ', '.join(f'{topic} {len(r)}/{rotations}' for topic, r in received.items()))
        print(f'{len(complete)} complete; {len(bad)} incomplete, mismatched or duplicate products; '
              f'{len(unexpected)} products with unexpected start stamps')
        if len(complete) < rotations:
            affected = [i + 1 for i, key in enumerate(stamps) if key not in complete]
            print('First affected turns (1-based):', affected[:10])
        if complete and not native_player:
            latency = [1000 * (max(r[key] for r in received.values()) - boundary[key]) for key in complete]
            print('Last required product after encoder wrap, p50/p95/max (ms):',
                  ' / '.join(f'{v:.1f}' for v in np.percentile(latency, [50, 95, 100])))
        print('Node CPU (s): ' + ', '.join(f'{name} {cpu_seconds(p)-cpu_start[name]:.2f}'
                                           for name, p in procs.items()))
        final = snapshot()
        print(f'Offered {len(packets)} spokes including the closing wrap packet.')
        for stage, keys in [('adapter', ('spokes', 'invalid_spokes')),
                            ('assembler', ('received_spokes', 'rejected_spokes', 'missing_bearing_cells',
                                           'buffer_resets', 'time_resets', 'discarded_buffered_spokes'))]:
            print(stage + ': ' + ', '.join(f'{k}={int(final[stage][k])-int(initial[stage][k])}'
                                           for k in keys))
        for name, process in procs.items():
            with Path(f'/proc/{process.pid}/status').open() as status:
                peak_kib = next(int(line.split()[1]) for line in status if line.startswith('VmHWM:'))
            print(f'{name} peak RSS: {peak_kib / 1024:.1f} MiB')
        if measured_rate is not None and measured_rate < 0.95 * hz:
            print('Measured rate was below 95% of the requested rate.')
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
    parser.add_argument('--native-player', action='store_true',
                        help='Stage a temporary MCAP and benchmark the installed C++ bag player through all outputs')
    args = parser.parse_args()
    if args.rotations < 1 or not math.isfinite(args.hz) or args.hz <= 0:
        parser.error('rotations and hz must be positive')
    raise SystemExit(0 if run(args.bag, args.rotations, args.hz, args.composed, args.synthetic, args.native_player) else 1)
