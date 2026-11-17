"""Compare native and Python executables over DDS, including partial rotations."""
from array import array
from contextlib import ExitStack
import math
from pathlib import Path
import subprocess
import time
from uuid import uuid4

from ament_index_python.packages import get_package_prefix
from compare_playback import compare, run_pipeline, stop
from nav_ross.msg import HighPrecisionFFTData, nav_msg
from navtech_radar_msgs.msg import FftSpoke, FftSweep, RadarConfiguration
import numpy as np
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2


@pytest.mark.parametrize(('mode', 'selection'), [('cfar', 'nearest'), ('threshold', 'strongest')])
def test_installed_pipelines(mode, selection):
    cfg = nav_msg(range_resolution=0.125, AzimuthSamples=4, EncoderSize=360,
                  BinSize=1250, RangeInBins=96, ExpectedRotationRate=4)

    def spoke(angle, tick, empty=False):
        levels = array('H', [1000] * 96)
        if not empty:
            # Adjacent equal-strength reflectors test suppression and peak budget.
            levels[24], levels[26], levels[60] = 35000, 35000, 65535
        m = HighPrecisionFFTData(angle=float(angle), azimuth=angle, size=96,
                                 data=levels, sweepCounter=tick)
        m.header.stamp.sec = 12
        m.header.stamp.nanosec = tick * 62500000
        return m

    partial = [spoke(90, 0), spoke(90, 1), spoke(270, 2, empty=True)]
    complete = [spoke(a, i + 3) for i, a in enumerate([0, 90, 180, 270])]
    sentinel = spoke(0, 7)
    # Each run launches different executables. Frozen sim time gives equal
    # host stamps so acquisition and receipt metadata can both be compared.
    with rclpy.init():
        results = [run_pipeline(impl, cfg, [partial, complete], sentinel,
                                selection, mode, ['max_points_per_spoke:=1',
                                                  'azimuth_offset_degrees:=90.0'])[0]
                   for impl in ('cpp', 'python')]
    compare(*results)
    _, scan, cloud = results[0][0]
    assert scan.angle_increment == pytest.approx(-math.pi / 2)
    assert cloud.width == 2
    assert scan.ranges[0] == (3.0 if selection == 'nearest' else 7.5)
    assert math.isnan(scan.ranges[1]) and math.isnan(scan.ranges[2])
    assert scan.time_increment == 0.0
    _, scan, cloud = results[0][1]
    assert scan.time_increment == 0.0625 and cloud.width == 4
    points = np.frombuffer(cloud.data, dtype='<f4').reshape(-1, 4)
    np.testing.assert_allclose(points[0], [0.0, 7.5, 0, 65535], atol=1e-5)
    np.testing.assert_allclose(points[1], [7.5, 0.0, 0, 65535], atol=1e-5)


@pytest.mark.parametrize('suffix', ['', '_py'])
def test_canonical_calibration_over_dds(suffix):
    cfg = RadarConfiguration(configuration_version=1, range_resolution=0.5, range_gain=2.0,
                             range_offset=1.0, range_bins=8, encoder_size=360, azimuth_samples=4,
                             expected_rotation_rate=4, sample_bit_depth=16)
    cfg.header.frame_id = 'radar'
    levels = [0, 0, 300, 0, 500, 0, 700, 0]
    rays = []
    for i, az in enumerate((0, 90, 180, 270, 0)):
        ray = FftSpoke(configuration_version=1, azimuth=az, azimuth_angle=math.radians(az),
                       data=levels, sample_bit_depth=16)
        ray.header.frame_id = 'radar'
        ray.header.stamp.sec = 10
        ray.header.stamp.nanosec = i * 62500000
        rays.append(ray)

    ns = '/calibration_' + uuid4().hex
    with rclpy.init(), ExitStack() as cleanup:
        node = rclpy.create_node('fixture', namespace=ns)
        cleanup.callback(node.destroy_node)
        configs = node.create_publisher(RadarConfiguration, 'navtech_radar/configuration',
                                        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        spokes = node.create_publisher(FftSpoke, 'navtech_radar/fft/spoke', 16)
        outputs = {}

        def collect(topic, message):
            if message.header.stamp == rays[0].header.stamp:
                outputs[topic] = message

        for topic, msg, qos in [('fft/sweep', FftSweep, 1), ('scan', LaserScan, qos_profile_sensor_data),
                                ('point_cloud', PointCloud2, qos_profile_sensor_data)]:
            node.create_subscription(msg, 'navtech_radar/' + topic,
                                     lambda value, key=topic: collect(key, value), qos)
        package = 'navtech_radar_processing'
        binary = Path(get_package_prefix(package)) / 'lib' / package / ('fft_sweep_assembler' + suffix)
        cmd = [str(binary), '--ros-args', '-r', '__ns:=' + ns]
        for param in ['publish_scan:=true', 'publish_point_cloud:=true', 'min_range:=0.0',
                      'max_range:=5.0', 'intensity_threshold:=100', 'peak_selection:=threshold']:
            cmd.extend(['-p', param])
        proc = subprocess.Popen(cmd)
        cleanup.callback(stop, proc)
        configs.publish(cfg)
        deadline = time.monotonic() + 12
        next_send = 0.0
        while len(outputs) < 3 and proc.poll() is None and time.monotonic() < deadline:
            if spokes.get_subscription_count() and time.monotonic() >= next_send:
                for ray in rays:
                    spokes.publish(ray)
                next_send = time.monotonic() + 0.25
            rclpy.spin_once(node, timeout_sec=0.02)
        assert set(outputs) == {'fft/sweep', 'point_cloud', 'scan'}
    sweep, scan = outputs['fft/sweep'], outputs['scan']
    assert sweep.range_resolution == 0.5
    np.testing.assert_array_equal(np.frombuffer(sweep.image.data, dtype='<u2').reshape(4, 8),
                                  [levels] * 4)
    assert (scan.range_min, scan.range_max) == (1.0, 5.0)
    np.testing.assert_array_equal(scan.ranges, [3.0] * 4)
    np.testing.assert_array_equal(scan.intensities, [300.0] * 4)
    assert scan.time_increment == 0.0625
    points = np.frombuffer(outputs['point_cloud'].data, dtype='<f4').reshape(4, 2, 4)
    directions = np.array([[1, 0], [0, -1], [-1, 0], [0, 1]])
    for column, radius in enumerate((3.0, 5.0)):
        np.testing.assert_allclose(points[:, column, :2], radius * directions, atol=1e-6)
