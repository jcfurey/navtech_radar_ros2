"""Record the driver's own eight-bit output, then replay it through the installed launch."""
from contextlib import ExitStack
import subprocess
import time

from compare_playback import stop
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TransformStamped
from navtech_radar_msgs.msg import RecordedRotation
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import StaticTransformBroadcaster
import yaml


def rotation(index, rows):
    msg = RecordedRotation()
    cfg = msg.configuration
    cfg.header.frame_id = 'navtech'
    cfg.header.stamp.sec = 10
    cfg.header.stamp.nanosec = index * 250_000_000
    cfg.configuration_version = 1
    cfg.azimuth_samples, cfg.encoder_size = 4, 360
    cfg.range_bins, cfg.range_resolution, cfg.sample_bit_depth = 96, 0.1, 8
    cfg.expected_rotation_rate = 4
    img = msg.image
    img.header = cfg.header
    img.width, img.height, img.step, img.encoding = 96, rows, 96, 'mono8'
    pixels = bytearray(96 * rows)
    for row in range(rows):
        pixels[96 * row + 30 + row] = 120 + index + row
    img.data = bytes(pixels)
    msg.encoder_values = [90*i for i in range(rows)]
    if index == 2:
        msg.sensor_ns = [10_500_000_000 + i*62_500_000 for i in range(rows)]
    return msg


def test_canonical_bag_round_trip(tmp_path):
    bag = tmp_path/'canonical'
    base = '/navtech_radar/'
    recorded = '/robot' + base
    with rclpy.init(), ExitStack() as cleanup:
        node = rclpy.create_node('canonical_bag_check')
        cleanup.callback(node.destroy_node)
        clouds, scans, reports = {}, {}, []
        node.create_subscription(PointCloud2, base+'point_cloud',
                                 lambda m: clouds.update({m.header.stamp.nanosec: m}), qos_profile_sensor_data)
        node.create_subscription(LaserScan, base+'scan',
                                 lambda m: scans.update({m.header.stamp.nanosec: m}), qos_profile_sensor_data)
        node.create_subscription(DiagnosticArray, '/diagnostics',
                                 lambda m: reports.extend(dict((v.key, v.value) for v in s.values)
                                                          for s in m.status if s.name.endswith(': sweep assembly')), 10)
        rotations = node.create_publisher(RecordedRotation, base+'recording/rotation', 4)
        broadcaster = StaticTransformBroadcaster(node)
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id, tf.transform.rotation.w = 'odom', 'navtech', 1.0
        broadcaster.sendTransform(tf)

        def wait(condition, proc):
            deadline = time.monotonic() + 18
            while not condition():
                assert proc.poll() is None and time.monotonic() < deadline, (clouds.keys(), scans.keys(), reports[-1:])
                rclpy.spin_once(node, timeout_sec=0.05)

        with ExitStack() as source:
            recorder = subprocess.Popen(['ros2', 'bag', 'record', '-o', str(bag), '--topics',
                                         recorded+'configuration', recorded+'fft/spoke', '--disable-keyboard-controls'])
            source.callback(stop, recorder)
            proc = subprocess.Popen(['ros2', 'run', 'navtech_radar_processing', 'fft_sweep_assembler',
                                     '--ros-args', '-p', 'recorded_input:=true', '-p', 'minimum_spokes:=1',
                                     '-p', 'publish_scan:=true',
                                     '-p', 'publish_point_cloud:=true', '-p', 'peak_selection:=cfar',
                                     '-r', base+'configuration:='+recorded+'configuration',
                                     '-r', base+'fft/spoke:='+recorded+'fft/spoke'])
            source.callback(stop, proc)
            wait(lambda: rotations.get_subscription_count() and node.count_subscribers(recorded+'fft/spoke')
                 and node.count_subscribers(recorded+'configuration') and node.count_publishers(base+'scan'), proc)
            for index, rows in enumerate((4, 1, 3)):
                rotations.publish(rotation(index, rows))
                wait(lambda: len(clouds) == len(scans) == index + 1, proc)
                assert clouds[index*250_000_000].width == rows
                assert scans[index*250_000_000].time_increment == (0.0625 if index == 2 else 0)
                if index == 0:
                    wait(lambda: reports and reports[-1]['sweep_timing'] == 'frame timestamp fallback', proc)
            wait(lambda: reports and reports[-1].get('received_spokes') == '8', proc)
            assert reports[-1]['missing_bearing_cells'] == '4'
        original_clouds, original_scans = clouds.copy(), scans.copy()
        info = yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
        counts = {e['topic_metadata']['name']: e['message_count'] for e in info['topics_with_message_count']}
        assert counts == {recorded+'configuration': 1, recorded+'fft/spoke': 8}

        for impl, deskew in [('cpp_composed', False), ('cpp', True)]:
            clouds.clear()
            scans.clear()
            reports.clear()
            with ExitStack() as playback:
                cmd = ['ros2', 'launch', 'navtech_radar_processing', 'playback.launch.py',
                       'bag:='+str(bag), 'implementation:='+impl]
                if deskew:
                    cmd.append('deskew_fixed_frame:=odom')
                proc = subprocess.Popen(cmd)
                playback.callback(stop, proc)
                wait(lambda: 500_000_000 in clouds and len(scans) == 3 and reports
                     and reports[-1].get('received_spokes') == '8', proc)
                assert reports[-1]['missing_bearing_cells'] == '4'
                assert reports[-1]['rejected_spokes'] == reports[-1]['buffer_resets'] == '0'
                assert int(reports[-1]['deskew_skipped_timing']) == (2 if deskew else 0)
                assert set(clouds) == ({500_000_000} if deskew else set(original_clouds))
                for stamp, cloud in clouds.items():
                    assert cloud == original_clouds[stamp]
                for stamp, scan in scans.items():
                    # Missing bearings carry IEEE NaN. Compare their binary representation.
                    assert scan.header == original_scans[stamp].header
                    assert scan.time_increment == original_scans[stamp].time_increment
                    assert scan.ranges.tobytes() == original_scans[stamp].ranges.tobytes()
                    assert scan.intensities == original_scans[stamp].intensities
