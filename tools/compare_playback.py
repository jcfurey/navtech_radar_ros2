#!/usr/bin/env python3
"""Compare both installed playback pipelines on the same recorded revolutions (Linux)."""
import argparse
import hashlib
import os
from pathlib import Path
import signal
import subprocess
import time
from contextlib import ExitStack

from ament_index_python.packages import get_package_prefix
from nav_ross.msg import HighPrecisionFFTData, nav_msg
from navtech_radar_msgs.msg import FftSpoke, FftSweep, RadarConfiguration
import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Image, LaserScan, PointCloud2


def recorded_input(bag, rotations):
    import rosbag2_py
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/Navtech/Configuration_Topic', '/Navtech/FFTData']))
    config = None
    groups, group = [], []
    while reader.has_next():
        topic, payload, *_ = reader.read_next_ext()
        if topic.endswith('Configuration_Topic'):
            incoming = deserialize_message(payload, nav_msg)
            if config is not None and config != incoming:
                raise ValueError('Benchmark requires a single configuration')
            config = incoming
        elif config is not None:
            spoke = deserialize_message(payload, HighPrecisionFFTData)
            if group and group[-1].angle - spoke.angle > 180:
                groups.append(group)
                group = []
                if len(groups) == rotations + 1:  # warmup + measured rotations
                    return config, groups, spoke
            group.append(spoke)
    raise ValueError('Recording ended before the requested revolutions')


def stop(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def cpu_seconds(process):
    # Fields after the command name start with state (field 3).
    fields = Path(f'/proc/{process.pid}/stat').read_text().rsplit(')', 1)[1].split()
    return (int(fields[11]) + int(fields[12])) / os.sysconf('SC_CLK_TCK')


def run_pipeline(implementation, config, groups, sentinel, selection='nearest', mode='cfar',
                 assembler_options=()):
    """Feed bounded bursts and wait for all four products at every revolution."""
    suffix = '_py' if implementation == 'python' else ''
    node = rclpy.create_node('playback_compare', namespace='/navtech_check')
    with ExitStack() as cleanup:
        cleanup.callback(node.destroy_node)
        cfg_pub = node.create_publisher(nav_msg, 'legacy_config', 1)
        fft_pub = node.create_publisher(HighPrecisionFFTData, 'legacy_fft', QoSProfile(depth=512))
        configs, outputs = [], {}
        node.create_subscription(RadarConfiguration, 'navtech_radar/configuration', configs.append,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        for key, kind, qos in [('fft/sweep', FftSweep, 2), ('fft/image', Image, 2),
                               ('scan', LaserScan, qos_profile_sensor_data),
                               ('point_cloud', PointCloud2, qos_profile_sensor_data)]:
            outputs[key] = []
            node.create_subscription(kind, 'navtech_radar/' + key, outputs[key].append, qos)

        processes = []
        for package, exe, params in [
                ('navtech_radar_processing', 'fft_sweep_assembler', [
                    'publish_scan:=true', 'publish_point_cloud:=true',
                    'peak_selection:=' + mode, 'scan_selection:=' + selection, *assembler_options]),
                ('navtech_radar_driver', 'legacy_bag_driver', [
                    'legacy_configuration_topic:=legacy_config', 'legacy_fft_topic:=legacy_fft'])]:
            binary = Path(get_package_prefix(package)) / 'lib' / package / (exe + suffix)
            args = [str(binary), '--ros-args', '-r', '__ns:=/navtech_check', '-p', 'use_sim_time:=true']
            for param in params:
                args.extend(['-p', param])
            process = subprocess.Popen(args)
            cleanup.callback(stop, process)
            processes.append(process)

        def wait_until(predicate):
            deadline = time.monotonic() + 12
            while not predicate():
                if time.monotonic() > deadline or any(p.poll() is not None for p in processes):
                    raise RuntimeError(f'{implementation} pipeline timed out: '
                                       f'{ {k: len(v) for k,v in outputs.items()} }')
                rclpy.spin_once(node, timeout_sec=0.01)

        wait_until(lambda: cfg_pub.get_subscription_count() == fft_pub.get_subscription_count() == 1
                   and node.count_subscribers('navtech_radar/fft/spoke') == 1
                   and node.count_subscribers('navtech_radar/configuration') >= 2
                   and all(node.count_publishers('navtech_radar/' + key) for key in outputs))
        # Repeating configuration before startup matches TAMU. Wait for the
        # actual reader matches (graph discovery can precede DDS matching).
        cfg_pub.publish(config)
        wait_until(lambda: configs)
        first_seen = []
        probe = node.create_subscription(FftSpoke, 'navtech_radar/fft/spoke', first_seen.append, qos_profile_sensor_data)
        wait_until(lambda: probe.get_publisher_count() == 1)
        # Fence on a pre-roll spoke; rewind before the real first spoke. The
        # assembler ignores a one-spoke partial revolution at the boundary.
        warmup = deserialize_message(serialize_message(groups[0][0]), HighPrecisionFFTData)
        warmup.header.stamp.sec += 1
        fft_pub.publish(warmup)
        wait_until(lambda: first_seen)
        node.destroy_subscription(probe)

        summaries = []
        wall_start, usage_start = time.monotonic(), [cpu_seconds(p) for p in processes]
        for n, rows in enumerate(groups):
            if n == 1:
                wall_start, usage_start = time.monotonic(), [cpu_seconds(p) for p in processes]
            boundary = groups[n+1][0] if n+1 < len(groups) else sentinel
            inputs = rows if n == 0 else rows[1:]
            for i, spoke in enumerate(inputs):
                fft_pub.publish(spoke)
                if i % 32 == 31:
                    time.sleep(0.001)
            fft_pub.publish(boundary)
            wait_until(lambda: all(outputs.values()))
            sweep, image, scan, cloud = (outputs[k].pop(0) for k in ('fft/sweep','fft/image','scan','point_cloud'))
            ordered = sorted(rows, key=lambda s: s.azimuth)
            assert sweep.image == image and sweep.header == scan.header == cloud.header
            assert sweep.sweep_sequence == n and image.width == config.RangeInBins
            assert sweep.header.stamp == rows[0].header.stamp and sweep.end_stamp == rows[-1].header.stamp
            assert list(sweep.encoder_values) == [s.azimuth for s in ordered]
            assert list(sweep.sweep_counters) == [s.sweepCounter for s in ordered]
            assert list(sweep.timestamps) == [s.header.stamp for s in ordered]
            assert image.data.tobytes() == b''.join(np.asarray(s.data, dtype='<u2').tobytes() for s in ordered)
            assert len(scan.ranges) == config.AzimuthSamples
            summaries.append((hashlib.sha256(serialize_message(sweep)).digest(), scan, cloud))
        usage = [cpu_seconds(p)-start for p,start in zip(processes,usage_start)]
        return summaries, usage, time.monotonic()-wall_start


def compare(left, right):
    assert len(left) == len(right)
    for (raw, scan, cloud), (other_raw, other_scan, other_cloud) in zip(left,right):
        assert raw == other_raw
        assert serialize_message(scan) == serialize_message(other_scan)
        assert cloud.fields == other_cloud.fields and cloud.width == other_cloud.width
        assert cloud.point_step == other_cloud.point_step == 16 and cloud.height == other_cloud.height == 1
        assert cloud.header == other_cloud.header and cloud.row_step == other_cloud.row_step
        a = np.frombuffer(cloud.data, dtype='>f4' if cloud.is_bigendian else '<f4').reshape(-1,4)
        b = np.frombuffer(other_cloud.data, dtype='>f4' if other_cloud.is_bigendian else '<f4').reshape(-1,4)
        np.testing.assert_allclose(a[:,:2], b[:,:2], rtol=2e-6, atol=2e-5)
        np.testing.assert_array_equal(a[:,2:], b[:,2:])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    parser.add_argument('--rotations', type=int, default=20)
    args = parser.parse_args()
    if args.rotations < 1:
        parser.error('--rotations must be positive')
    inputs = recorded_input(args.bag, args.rotations)
    with rclpy.init():
        reference, py_cpu, py_wall = run_pipeline('python', *inputs)
        native, cpp_cpu, cpp_wall = run_pipeline('cpp', *inputs)
    compare(reference, native)
    print(f'{args.rotations} measured revolutions; raw sweep and scan parity, cloud XYZ/intensity checked.')
    for label, cpu, wall in [('Python', py_cpu, py_wall), ('C++', cpp_cpu, cpp_wall)]:
        print(f'{label}: assembler {cpu[0]:.2f}s CPU, adapter {cpu[1]:.2f}s CPU, {wall:.2f}s wall')
    print(f'Combined node CPU ratio: {sum(py_cpu)/sum(cpp_cpu):.2f}x')


if __name__ == '__main__':
    main()
