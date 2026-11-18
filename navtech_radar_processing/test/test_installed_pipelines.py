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
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
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


@pytest.mark.parametrize('suffix', ['', '_py'])
def test_strict_sweeps_and_motion_compensation(suffix):
    # Each accepted ray measures 2 m. TF moves the radar by (3, 0, 3) m
    # and turns it 90 degrees relative to its initial, already rotated pose.
    ns = '/deskew_' + uuid4().hex
    radar, fixed = ns[1:] + '_radar', ns[1:] + '_odom'
    cfg = RadarConfiguration(configuration_version=1, range_resolution=0.5, range_bins=8,
                             encoder_size=360, azimuth_samples=4, expected_rotation_rate=4,
                             sample_bit_depth=16)
    cfg.header.frame_id = radar
    with rclpy.init(), ExitStack() as cleanup:
        node = rclpy.create_node('fixture', namespace=ns)
        cleanup.callback(node.destroy_node)
        config_pub = node.create_publisher(RadarConfiguration, 'navtech_radar/configuration',
                                           QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        pub = node.create_publisher(FftSpoke, 'navtech_radar/fft/spoke', 64)
        tf = TransformBroadcaster(node)
        received = {'fft/sweep': {}, 'point_cloud': {}, 'scan': {}}
        for topic, msg, qos in [('fft/sweep', FftSweep, 8), ('point_cloud', PointCloud2, qos_profile_sensor_data),
                                ('scan', LaserScan, qos_profile_sensor_data)]:
            node.create_subscription(msg, 'navtech_radar/' + topic,
                                     lambda m, key=topic: received[key].update({m.header.stamp.sec: m}), qos)
        package = 'navtech_radar_processing'
        exe = Path(get_package_prefix(package)) / 'lib' / package / ('fft_sweep_assembler' + suffix)
        cmd = [str(exe), '--ros-args', '-r', '__ns:=' + ns]
        for option in ['publish_scan:=true', 'publish_point_cloud:=true', 'publish_partial_sweeps:=false',
                       'deskew_fixed_frame:=' + fixed, 'min_range:=0.0']:
            cmd += ['-p', option]
        child = subprocess.Popen(cmd)
        cleanup.callback(stop, child)
        config_pub.publish(cfg)

        def wait(condition, retry=None):
            deadline = time.monotonic() + 10
            again = 0.0
            while not condition():
                assert child.poll() is None and time.monotonic() < deadline, received.keys()
                if retry and time.monotonic() >= again:
                    retry()
                    again = time.monotonic() + 0.2
                rclpy.spin_once(node, timeout_sec=0.01)

        def send(sec, azimuths=(0, 90, 180, 270, 0), *, angles=None, seconds=None):
            for i, az in enumerate(azimuths):
                ray = FftSpoke(configuration_version=1, azimuth=az, sample_bit_depth=16,
                               azimuth_angle=math.radians(az if angles is None else angles[i]),
                               data=[0, 0, 0, 0, 30000, 0, 0, 0])
                ray.header.frame_id = radar
                ray.header.stamp.sec = sec if seconds is None else seconds[i]
                ray.header.stamp.nanosec = i * 62500000
                pub.publish(ray)

        def pose(sec, last):
            m = TransformStamped()
            m.header.frame_id, m.child_frame_id = fixed, radar
            m.header.stamp.sec = sec
            m.header.stamp.nanosec = 187500000 if last else 0
            yaw = math.pi if last else math.pi / 2
            m.transform.rotation.z, m.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            m.transform.translation.x = 10.0
            m.transform.translation.y = 3.0 if last else 0.0
            m.transform.translation.z = 4.0 if last else 1.0
            return m

        wait(lambda: pub.get_subscription_count() and node.count_subscribers('/tf'))
        wait(lambda: 100 in received['scan'], lambda: send(100))
        send(1, seconds=[1, 1, 2, 2, 2])
        send(3, azimuths=[0, 1, 180, 270, 0])
        send(4, angles=[0, 180, 180, 270, 0])
        send(0)
        wait(lambda: 0 in received['scan'])
        tf.sendTransform(pose(10, False))
        send(10)
        wait(lambda: 10 in received['fft/sweep'] and 10 in received['scan'])
        for topic in ('fft/sweep', 'scan'):
            assert not ({1, 2, 3, 4} & received[topic].keys())
        assert 10 not in received['point_cloud']
        wait(lambda: 10 in received['point_cloud'],
             lambda: tf.sendTransform([pose(10, False), pose(10, True)]))
        assert 0 not in received['point_cloud']
        cloud = received['point_cloud'][10]
        assert cloud.header == received['fft/sweep'][10].header
        np.testing.assert_array_equal(received['scan'][10].ranges, [2] * 4)
        phi = np.deg2rad(np.arange(4) * (30 - 90))
        expected = np.column_stack((np.arange(4) + 2*np.cos(phi), 2*np.sin(phi),
                                    np.arange(4), np.full(4, 30000)))
        points = np.frombuffer(cloud.data, dtype='<f4').reshape(-1, 4)
        np.testing.assert_allclose(points, expected, atol=1e-5)


def test_packet_loss_across_encoder_zero():
    cfg = nav_msg(range_resolution=0.5, AzimuthSamples=8, EncoderSize=360,
                  BinSize=5000, RangeInBins=8, ExpectedRotationRate=4)

    def recorded_spoke(tick):
        angle = (tick % 8) * 45
        levels = array('H', [0, 0, 0, 0, 30000 + tick, 0, 0, 0])
        ray = HighPrecisionFFTData(angle=float(angle), azimuth=angle,
                                   sweepCounter=(65530 + tick) % 65536, size=8, data=levels)
        ray.header.stamp.sec = 10
        ray.header.stamp.nanosec = tick * 31250000
        return ray

    # Lose five spokes around north and cross the uint16 counter rollover.
    # Azimuth drops from 180 to 90 degrees, inside the ordinary wrap threshold.
    groups = [[recorded_spoke(tick) for tick in ticks] for ticks in (range(5), range(10, 16))]
    with rclpy.init():
        results = [run_pipeline(impl, cfg, groups, recorded_spoke(16), mode='threshold')[0]
                   for impl in ('cpp', 'python')]
    compare(*results)
    for ((sweep, _), scan, cloud), covered in zip(results[0], [5, 6]):
        assert len(sweep.timestamps) == cloud.width == covered
        assert sum(math.isfinite(r) for r in scan.ranges) == covered


def test_file_launch_keeps_chirps_and_publishes_last_partial(tmp_path):
    import cv2

    folder = tmp_path/'boreas-2025-07-18-10-00'/'radar'
    folder.mkdir(parents=True)
    first_usec = 1752847243464332
    encoders = np.array([2800, 4200, 0], dtype='<u2')
    expected = []
    for i in range(2):
        image = np.zeros((3, 11+96), np.uint8)
        usec = first_usec + i*250000 + np.arange(3, dtype='<u8')*625
        image[:, :8] = usec.view(np.uint8).reshape(3, 8)
        image[:, 8:10] = encoders.view(np.uint8).reshape(3, 2)
        image[:, 10] = [0, 255, 0]
        image[:, 11+80] = [90+i, 120+i, 240+i]
        assert cv2.imwrite(str(folder/f'{usec[0]}.png'), image)
        expected.append(image[[2, 0, 1], 11:])

    with rclpy.init(), ExitStack() as cleanup:
        node = rclpy.create_node('fixture_' + uuid4().hex)
        cleanup.callback(node.destroy_node)
        received = []
        node.create_subscription(FftSweep, '/navtech_radar/fft/sweep', received.append, 4)
        child = subprocess.Popen(['ros2', 'launch', 'navtech_radar_processing', 'playback.launch.py',
                                  'recording:=' + str(folder)])
        cleanup.callback(stop, child)
        deadline = time.monotonic() + 15
        while len(received) < 2 and time.monotonic() < deadline and child.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert len(received) == len(expected)
        for i, (sweep, rows) in enumerate(zip(received, expected)):
            assert sweep.sample_bit_depth == 8 and sweep.image.encoding == 'mono16'
            assert sweep.range_resolution == pytest.approx(0.04381)
            assert list(sweep.encoder_values) == [0, 2800, 4200]
            assert sweep.header.stamp.sec*10**9 + sweep.header.stamp.nanosec == (first_usec+i*250000)*1000
            np.testing.assert_array_equal(np.frombuffer(sweep.image.data, '<u2').reshape(3, 96), rows)
