#!/usr/bin/env python3
"""Compare both installed playback pipelines on the same recorded revolutions (Linux)."""
import argparse
from contextlib import ExitStack
from copy import deepcopy
import hashlib
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from uuid import uuid4

from ament_index_python.packages import get_package_prefix
from nav_ross.msg import HighPrecisionFFTData, nav_msg
from navtech_radar_msgs.msg import FftSweep, RadarConfiguration
import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
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


def comparison_command(package, name, reference=False):
    """Python comparison oracles live only in the source checkout."""
    if reference:
        return [sys.executable, str(Path(__file__).resolve().parents[1] / package / 'test/reference' / (name + '.py'))]
    return [str(Path(get_package_prefix(package)) / 'lib' / package / name)]


def run_pipeline(implementation, config, groups, sentinel, selection='nearest', mode='cfar',
                 assembler_options=()):
    """Feed bounded bursts and wait for all four products at every revolution."""
    namespace = f'/navtech_check_{uuid4().hex}'
    node = rclpy.create_node('playback_compare', namespace=namespace)
    # Mark discovery traffic with a time beyond the first acquisition. The
    # first recorded spoke then rewinds both assemblers to an empty buffer.
    low = deepcopy(groups[0][0])
    low.header.stamp.sec = min(sentinel.header.stamp.sec + 1, 2147483647)
    low.header.stamp.nanosec = 999999999
    low.azimuth = 0
    low.angle = 0.0
    high = deepcopy(low)
    high.azimuth = round(0.75 * config.EncoderSize) % config.EncoderSize
    high.angle = high.azimuth * 360.0 / config.EncoderSize
    with ExitStack() as cleanup:
        cleanup.callback(node.destroy_node)
        cfg_pub = node.create_publisher(nav_msg, 'legacy_config', 1)
        fft_pub = node.create_publisher(HighPrecisionFFTData, 'legacy_fft', QoSProfile(depth=512))
        configs, outputs, probe_products = [], {}, set()
        node.create_subscription(RadarConfiguration, 'navtech_radar/configuration', configs.append,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        def collect(key, message):
            if message.header.stamp == low.header.stamp:
                probe_products.add(key)
            else:
                outputs[key].append(message)

        for key, kind, qos in [('fft/sweep', FftSweep, 2), ('fft/image', Image, 2),
                               ('scan', LaserScan, qos_profile_sensor_data),
                               ('point_cloud', PointCloud2, qos_profile_sensor_data)]:
            outputs[key] = []
            node.create_subscription(kind, 'navtech_radar/' + key,
                                     lambda msg, key=key: collect(key, msg), qos)

        processes = []
        for package, exe, params in [
                ('navtech_radar_processing', 'fft_sweep_assembler', [
                    'publish_scan:=true', 'publish_point_cloud:=true',
                    'peak_selection:=' + mode, 'scan_selection:=' + selection, *assembler_options]),
                ('navtech_radar_driver', 'legacy_bag_driver', [
                    'legacy_configuration_topic:=legacy_config', 'legacy_fft_topic:=legacy_fft'])]:
            binary = comparison_command(package, exe, implementation == 'python')
            args = [*binary, '--ros-args', '-r', '__ns:=' + namespace, '-p', 'use_sim_time:=true']
            for param in params:
                args.extend(['-p', param])
            process = subprocess.Popen(args)
            cleanup.callback(stop, process)
            processes.append(process)

        def wait_until(predicate, description, retry=None):
            deadline = time.monotonic() + 12
            next_retry = 0
            while not predicate():
                now = time.monotonic()
                if now > deadline or any(p.poll() is not None for p in processes):
                    raise RuntimeError(f'{implementation} pipeline failed while waiting for {description}')
                if retry and now >= next_retry:
                    retry()
                    next_retry = now + 0.25
                rclpy.spin_once(node, timeout_sec=0.01)

        wait_until(lambda: cfg_pub.get_subscription_count() == fft_pub.get_subscription_count() == 1
                   and node.count_subscribers('navtech_radar/fft/spoke') == 1
                   and node.count_subscribers('navtech_radar/configuration') >= 2
                   and all(node.count_publishers('navtech_radar/' + key) for key in outputs),
                   'endpoint discovery')
        wait_until(lambda: configs, 'configuration', lambda: cfg_pub.publish(config))

        def publish_probe():
            for spoke in (low, high, low):
                fft_pub.publish(spoke)

        wait_until(lambda: probe_products == outputs.keys(), 'all four probe products', publish_probe)

        summaries = []
        first_sequence = None
        wall_start, usage_start = time.monotonic(), [cpu_seconds(p) for p in processes]
        for n, rows in enumerate(groups):
            if n == 1:
                wall_start, usage_start = time.monotonic(), [cpu_seconds(p) for p in processes]
            boundary = groups[n + 1][0] if n + 1 < len(groups) else sentinel
            inputs = rows if n == 0 else rows[1:]
            for i, spoke in enumerate(inputs):
                fft_pub.publish(spoke)
                if i % 32 == 31:
                    time.sleep(0.001)
            fft_pub.publish(boundary)
            wait_until(lambda: all(outputs.values()), f'products for recorded revolution {n}')
            sweep, image, scan, cloud = (outputs[k].pop(0) for k in
                                         ('fft/sweep', 'fft/image', 'scan', 'point_cloud'))
            ordered = sorted(rows, key=lambda s: s.azimuth)
            assert sweep.image == image and sweep.header == scan.header == cloud.header
            if first_sequence is None:
                first_sequence = sweep.sweep_sequence
            assert sweep.sweep_sequence == (first_sequence + n) % (1 << 32)
            assert image.width == config.RangeInBins
            assert sweep.header.stamp == rows[0].header.stamp and sweep.end_stamp == rows[-1].header.stamp
            assert list(sweep.encoder_values) == [s.azimuth for s in ordered]
            assert list(sweep.sweep_counters) == [s.sweepCounter for s in ordered]
            assert list(sweep.timestamps) == [s.header.stamp for s in ordered]
            assert image.data.tobytes() == b''.join(np.asarray(s.data, dtype='<u2').tobytes() for s in ordered)
            assert len(scan.ranges) == config.AzimuthSamples
            # Retain every field while bounding memory on long recordings.
            # DDS alignment padding is transport-specific, outside these fields.
            image_hash = hashlib.sha256(sweep.image.data).digest()
            sweep.image.data = []
            sweep.sweep_sequence = n
            summaries.append(((sweep, image_hash), scan, cloud))
        usage = [cpu_seconds(p) - start for p, start in zip(processes, usage_start)]
        return summaries, usage, time.monotonic() - wall_start


def compare(left, right):
    assert len(left) == len(right)
    for (raw, scan, cloud), (other_raw, other_scan, other_cloud) in zip(left, right):
        assert raw == other_raw
        for field in scan.get_fields_and_field_types():
            a, b = getattr(scan, field), getattr(other_scan, field)
            if field in ('ranges', 'intensities'):
                assert a.tobytes() == b.tobytes(), field
            else:
                assert a == b, field
        assert cloud.fields == other_cloud.fields and cloud.width == other_cloud.width
        assert cloud.point_step == other_cloud.point_step == 16 and cloud.height == other_cloud.height == 1
        assert cloud.header == other_cloud.header and cloud.row_step == other_cloud.row_step
        assert cloud.is_dense == other_cloud.is_dense
        a = np.frombuffer(cloud.data, dtype='>f4' if cloud.is_bigendian else '<f4').reshape(-1, 4)
        b = np.frombuffer(other_cloud.data, dtype='>f4' if other_cloud.is_bigendian else '<f4').reshape(-1, 4)
        np.testing.assert_allclose(a[:, :2], b[:, :2], rtol=2e-6, atol=2e-5)
        np.testing.assert_array_equal(a[:, 2:], b[:, 2:])


def check_boundary_loss(bag):
    """Remove a contiguous burst across one known wrap and verify both resulting sweeps."""
    config, groups, sentinel = recorded_input(bag, 2)
    first, second = groups[1:]
    count = config.AzimuthSamples
    if count < 4 or len(first) != count or len(second) != count:
        raise ValueError('Boundary-loss check needs two complete revolutions after warmup')
    rows = first + second
    if any((b.sweepCounter - a.sweepCounter) % 65536 != 1 for a, b in zip(rows, rows[1:])):
        raise ValueError('Boundary-loss check needs a continuous input packet sequence')
    parts = [first[:count // 2 + 1], second[count // 4:]]
    last_old, first_new = parts[0][-1], parts[1][0]
    if not 0 < last_old.angle - first_new.angle < 180:
        raise ValueError('Input lacks the expected encoder phases for the boundary-loss check')
    with rclpy.init():
        results = [run_pipeline(impl, config, parts, sentinel)[0] for impl in ('cpp', 'python')]
    compare(*results)
    n_old, n_new = map(len, parts)
    print(f'Omitted {2 * count - n_old - n_new} recorded spokes around encoder zero. '
          f'Both implementations emitted separate {n_old}- and {n_new}-row sweeps.')


def check_filtered_cloud(cloud, fft, encoders, encoder_size, spacing, offset=0.,
                         *, eight_bit=False, cfar=True):
    """Predict the complete default planar cloud from the source rows, including its point budget."""
    from sensor_msgs.msg import PointField
    from sensor_msgs_py.point_cloud2 import read_points

    names = ('x', 'y', 'z', 'intensity')
    assert [(f.name, f.offset, f.datatype, f.count) for f in cloud.fields] == [
        (name, 4*i, PointField.FLOAT32, 1) for i, name in enumerate(names)]
    assert cloud.height == 1 and cloud.point_step == 16 and cloud.is_dense
    assert len(cloud.data) == cloud.row_step == 16*cloud.width

    threshold, excess = (24, 12.) if eight_bit else (22000, 3000.)
    distances = np.arange(fft.shape[1])*spacing + offset
    window = np.flatnonzero((distances >= 2.) & (distances <= 200.))
    margin = 20 if cfar else 1
    window = window[(window >= margin) & (window < fft.shape[1] - margin)]
    training = np.r_[np.arange(-20, -4), np.arange(5, 21)]
    expected = []
    for ray in np.argsort(encoders, kind='stable'):
        row = fft[ray]
        levels = row[window]
        peaks = window[(levels >= threshold) & (levels > row[window-1]) & (levels >= row[window+1])]
        if cfar:
            background = row[peaks[:, None] + training].mean(axis=1)
            peaks = peaks[row[peaks] > background + excess]
        # Select strongest first, break ties by range, and suppress neighbours
        # directly in metres. The wire order within each spoke is near to far.
        ranked = peaks[np.lexsort((peaks, -row[peaks].astype(int)))]
        selected = []
        while ranked.size and len(selected) < 48:
            best = ranked[0]
            selected.append(best)
            ranked = ranked[1:]
            if cfar:
                ranked = ranked[np.abs(ranked - best)*spacing >= 0.5]
        if selected:
            bins = np.sort(selected)
            r = distances[bins]
            angle = -2*np.pi*int(encoders[ray])/encoder_size
            expected.append(np.column_stack((r*np.cos(angle), r*np.sin(angle), row[bins])))

    reference = np.concatenate(expected) if expected else np.empty((0, 3))
    points = read_points(cloud, skip_nans=False)
    assert len(reference) == len(points) == cloud.width
    assert np.all(points['z'] == 0.)
    np.testing.assert_array_equal(points['intensity'], reference[:, 2])
    xy = np.column_stack((points['x'], points['y'])).astype(float)
    np.testing.assert_allclose(xy, reference[:, :2], atol=2e-5, rtol=2e-7)
    return np.linalg.norm(xy - reference[:, :2], axis=1).max(initial=0.)


def check_recorded_exports(config, groups, results):
    """Read native ROS exports independently against turns with one recorded spoke per scan cell."""
    from laser_geometry import LaserProjection
    from sensor_msgs_py.point_cloud2 import read_points

    projector = LaserProjection()
    total_cloud = total_scan = 0
    max_cloud_error = max_scan_error = 0.
    count, resolution = config.AzimuthSamples, config.range_resolution
    step = 2 * np.pi / count
    for rows, (_, scan, cloud) in zip(groups, results):
        native = np.array([s.azimuth for s in rows]) * (2 * np.pi / config.EncoderSize)
        positions = (native - native[0]) % (2 * np.pi) / step
        cells = np.rint(positions).astype(int) % count
        if (len(rows) != count or len(np.unique(cells)) != count or
                np.max(np.abs(positions - np.rint(positions))) > 0.25):
            raise ValueError('Source check needs a complete turn with one spoke per scan cell')
        order = np.argsort(cells)
        fft = np.array([rows[i].data for i in order])

        def source_xy(ray, distance, intensity, angle):
            bins = np.rint(distance / resolution).astype(int)
            assert np.all((bins >= 0) & (bins < fft.shape[1]))
            np.testing.assert_array_equal(fft[ray, bins], intensity)
            radius = bins * resolution
            np.testing.assert_allclose(distance, radius, rtol=2e-6, atol=2e-5)
            return np.column_stack((radius * np.cos(angle), radius * np.sin(angle)))

        assert cloud.header == scan.header and cloud.header.frame_id == 'navtech'
        assert cloud.header.stamp == rows[0].header.stamp
        error = check_filtered_cloud(cloud, fft, [rows[i].azimuth for i in order],
                                     config.EncoderSize, resolution)
        max_cloud_error = max(max_cloud_error, error)
        total_cloud += cloud.width

        ranges = np.asarray(scan.ranges, dtype=float)
        valid = np.flatnonzero(np.isfinite(ranges))
        assert np.all((ranges[valid] >= scan.range_min) & (ranges[valid] <= scan.range_max))
        expected = source_xy(valid, ranges[valid], np.asarray(scan.intensities)[valid],
                             -native[0] - valid * step)
        projected = projector.projectLaser(scan)
        assert projected.header == scan.header
        points = read_points(projected)
        # laser_geometry uses an exclusive upper range cutoff.
        within = ranges[valid] < scan.range_max
        np.testing.assert_array_equal(points['index'], valid[within])
        xy = np.column_stack((points['x'], points['y']))
        np.testing.assert_allclose(xy, expected[within], rtol=2e-6, atol=1e-4)
        np.testing.assert_array_equal(points['intensity'], np.asarray(scan.intensities)[valid[within]])
        assert np.all(points['z'] == 0)
        max_scan_error = max(max_scan_error, np.linalg.norm(xy - expected[within], axis=1).max(initial=0.))
        total_scan += len(points)
    print(f'{total_cloud} cloud returns and {total_scan} projected scan returns checked against source FFT. '
          f'Maximum XY residual: {max_cloud_error:.7f} m on encoder bearings, '
          f'{max_scan_error:.7f} m on the scan grid.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--rotations', type=int, default=20)
    mode.add_argument('--check-boundary-loss', action='store_true',
                      help='Test loss of a burst across encoder zero on two recorded revolutions')
    parser.add_argument('--check-exports', action='store_true',
                        help='Check ROS consumer geometry and raw FFT intensities on complete recorded turns')
    args = parser.parse_args()
    if args.check_boundary_loss:
        if args.check_exports:
            parser.error('--check-exports requires complete recorded revolutions')
        check_boundary_loss(args.bag)
        return
    if args.rotations < 1:
        parser.error('--rotations must be positive')
    inputs = recorded_input(args.bag, args.rotations)
    with rclpy.init():
        reference, py_cpu, py_wall = run_pipeline('python', *inputs)
        native, cpp_cpu, cpp_wall = run_pipeline('cpp', *inputs)
    compare(reference, native)
    if args.check_exports:
        check_recorded_exports(inputs[0], inputs[1][1:], native[1:])
    print(f'{args.rotations} measured revolutions; raw sweep and scan parity, cloud XYZ/intensity checked.')
    for label, cpu, wall in [('Python', py_cpu, py_wall), ('C++', cpp_cpu, cpp_wall)]:
        print(f'{label}: assembler {cpu[0]:.2f}s CPU, adapter {cpu[1]:.2f}s CPU, {wall:.2f}s wall')
    print(f'Combined node CPU ratio: {sum(py_cpu)/sum(cpp_cpu):.2f}x')


if __name__ == '__main__':
    main()
