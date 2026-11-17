#!/usr/bin/env python3
"""Compare both installed visualizers on identical recorded sweeps (Linux)."""
import argparse
from contextlib import ExitStack
from copy import deepcopy
from pathlib import Path
import subprocess
import time
from uuid import uuid4

from ament_index_python.packages import get_package_prefix
from navtech_radar_msgs.msg import FftSweep, RadarConfiguration
import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image

from compare_playback import cpu_seconds, recorded_input, stop


class ImageComparison:
    """Publish one sweep at a time and collect both views from both executables."""

    def __init__(self, cleanup, parameters=(), views=('polar', 'cartesian')):
        ns = '/image_check_' + uuid4().hex
        self.node = rclpy.create_node('compare_images', namespace=ns)
        cleanup.callback(self.node.destroy_node)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.configs = self.node.create_publisher(RadarConfiguration, 'configuration', qos)
        self.sweeps = self.node.create_publisher(FftSweep, 'sweep', 1)
        self.outputs = {}
        self.views = views
        self.processes = {}
        directory = Path(get_package_prefix('navtech_radar_processing')) / 'lib/navtech_radar_processing'
        for impl, suffix in [('cpp', ''), ('python', '_py')]:
            cmd = [str(directory / ('fft_sweep_visualizer' + suffix)), '--ros-args',
                   '-r', f'__ns:={ns}/{impl}', '-p', 'sweep_topic:=' + ns + '/sweep',
                   '-p', 'configuration_topic:=' + ns + '/configuration']
            for param in parameters:
                cmd.extend(['-p', param])
            for view, topic in [('polar', 'display'), ('cartesian', 'cartesian')]:
                if view not in views:
                    continue
                key = impl, view
                self.node.create_subscription(
                    Image, f'{impl}/navtech_radar/fft/{topic}',
                    lambda msg, key=key: self.outputs.update({key: msg}), 1)
            child = subprocess.Popen(cmd)
            self.processes[impl] = child
            cleanup.callback(stop, child)

    def frame(self, sweep, *, warmup=False, ready=None):
        """Retry a startup probe until DDS, configuration and optional TF are ready."""
        self.outputs.clear()
        deadline = time.monotonic() + 15
        next_send = 0
        while time.monotonic() < deadline and all(p.poll() is None for p in self.processes.values()):
            now = time.monotonic()
            if now >= next_send:
                self.sweeps.publish(sweep)
                next_send = now + 0.3 if warmup else float('inf')
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if (len(self.outputs) != 2 * len(self.views)
                    or any(m.header != sweep.header for m in self.outputs.values())):
                continue
            if ready is None or ready(self.outputs):
                break
        else:
            raise RuntimeError(f'No matching images for input stamp {sweep.header.stamp}')
        images = {}
        for view in self.views:
            cpp = self.outputs['cpp', view]
            python = self.outputs['python', view]
            assert cpp.width == python.width and cpp.height == python.height, view
            assert cpp.encoding == python.encoding == 'bgr8', view
            assert cpp.step == python.step == 3 * cpp.width, view
            assert cpp.is_bigendian == python.is_bigendian == 0, view
            assert len(cpp.data) == len(python.data) == cpp.step * cpp.height, view
            if cpp.data != python.data:
                differences = np.count_nonzero(np.frombuffer(cpp.data, np.uint8) !=
                                               np.frombuffer(python.data, np.uint8))
                raise AssertionError(f'{view}: {differences} differing channel values at {sweep.header.stamp}')
            images[view] = cpp
        return images


def make_sweep(config, spokes, sequence):
    rows = sorted(spokes, key=lambda s: s.azimuth)
    sweep = FftSweep(configuration_version=config.configuration_version,
                     range_resolution=config.range_resolution, sweep_sequence=sequence, sample_bit_depth=16)
    sweep.header.frame_id = config.header.frame_id
    sweep.header.stamp = spokes[0].header.stamp
    sweep.end_stamp = spokes[-1].header.stamp
    sweep.encoder_values = [s.azimuth for s in rows]
    sweep.sweep_counters = [s.sweepCounter for s in rows]
    sweep.timestamps = [s.header.stamp for s in rows]
    sweep.host_stamps = sweep.timestamps
    sweep.image = Image(header=sweep.header, encoding='mono16', width=config.range_bins,
                        height=len(rows), step=2 * config.range_bins)
    sweep.image.data = b''.join(np.asarray(s.data, dtype='<u2').tobytes() for s in rows)
    return sweep


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    parser.add_argument('--rotations', type=int, default=32)
    args = parser.parse_args()
    if args.rotations < 1:
        parser.error('--rotations must be positive')
    legacy, groups, _ = recorded_input(args.bag, args.rotations)
    config = RadarConfiguration(configuration_version=1, range_resolution=legacy.range_resolution,
                                range_bins=legacy.RangeInBins, encoder_size=legacy.EncoderSize,
                                azimuth_samples=legacy.AzimuthSamples,
                                expected_rotation_rate=legacy.ExpectedRotationRate, sample_bit_depth=16)
    config.header.frame_id = 'navtech'
    with rclpy.init(), ExitStack() as cleanup:
        check = ImageComparison(cleanup)
        check.configs.publish(config)
        partial = make_sweep(config, groups[0], 0)
        probe = deepcopy(partial)
        probe.header.stamp.nanosec ^= 1
        probe.image.header = probe.header
        check.frame(probe, warmup=True)
        # A distinct stamp fences any repeated discovery probes before timing.
        check.frame(partial)
        start = {impl: cpu_seconds(p) for impl, p in check.processes.items()}
        for n, rows in enumerate(groups[1:], 1):
            check.frame(make_sweep(config, rows, n))
        for impl, p in check.processes.items():
            print(f'{impl}: {cpu_seconds(p) - start[impl]:.2f} s visualizer CPU')
    print(f'Both annotated BGR8 views matched on {args.rotations} measured revolutions.')


if __name__ == '__main__':
    main()
