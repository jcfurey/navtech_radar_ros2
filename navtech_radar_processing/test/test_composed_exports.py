"""Exercise the installed composed launch and read its exports with ROS consumer libraries."""
from array import array
from contextlib import ExitStack
import subprocess
import time

from compare_playback import stop
from cv_bridge import CvBridge
from laser_geometry import LaserProjection
from nav_ross.msg import HighPrecisionFFTData, nav_msg
from navtech_radar_msgs.msg import FftSweep
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


def test_composed_cardinal_returns():
    cfg = nav_msg(range_resolution=0.125, AzimuthSamples=4, EncoderSize=360,
                  BinSize=1250, RangeInBins=513, ExpectedRotationRate=4)
    # Unequal ranges distinguish handedness and all four cardinal directions.
    bins = np.array([120, 200, 264, 304])
    amplitudes = np.array([41000, 42000, 43000, 44000])
    raw = np.zeros((4, cfg.RangeInBins), dtype='<u2')
    for row, col in enumerate(bins):
        raw[row, col:col + 2] = amplitudes[row]
    # Exercise dense CFAR and equal-strength suppression within 0.5 m.
    raw[3, 352:464:2] = 30000
    rays = []
    for tick in range(5):
        row = tick % 4
        msg = HighPrecisionFFTData(angle=90.0 * row, azimuth=90 * row, sweepCounter=(65534 + tick) % 65536,
                                   size=cfg.RangeInBins, data=array('H', raw[row]))
        msg.header.stamp.sec = 10
        msg.header.stamp.nanosec = tick * 62_500_000
        rays.append(msg)

    with rclpy.init(), ExitStack() as cleanup:
        node = rclpy.create_node('composed_export_check')
        cleanup.callback(node.destroy_node)
        configs = node.create_publisher(nav_msg, '/Navtech/Configuration_Topic', 1)
        spokes = node.create_publisher(HighPrecisionFFTData, '/Navtech/FFTData', 16)
        outputs = {}
        topics = {'fft/sweep': FftSweep, 'fft/image': Image, 'fft/display': Image,
                  'fft/cartesian': Image, 'point_cloud': PointCloud2, 'scan': LaserScan}
        def collect(key, message):
            if message.header.stamp == rays[0].header.stamp:
                outputs[key] = message

        for topic, kind in topics.items():
            qos = qos_profile_sensor_data if topic in ('point_cloud', 'scan') else 2
            node.create_subscription(kind, '/navtech_radar/' + topic,
                                     lambda msg, key=topic: collect(key, msg), qos)
        proc = subprocess.Popen(['ros2', 'launch', 'navtech_radar_processing', 'playback.launch.py',
                                 'implementation:=cpp_composed', 'use_sim_time:=false'])
        cleanup.callback(stop, proc)

        def collect_turn(warmup):
            outputs.clear()
            deadline = time.monotonic() + 15
            retry = 0
            while outputs.keys() != topics.keys():
                now = time.monotonic()
                assert now < deadline and proc.poll() is None, topics.keys() - outputs.keys()
                if now >= retry:
                    if warmup:
                        configs.publish(cfg)
                    for ray in rays:
                        spokes.publish(ray)
                    retry = now + 0.25 if warmup else float('inf')
                rclpy.spin_once(node, timeout_sec=0.01)

        collect_turn(warmup=True)
        for ray in rays:
            ray.header.stamp.sec = 11
        # Counter rollover and a 100 µs stamp inversion inside the measured turn.
        rays[2].header.stamp.nanosec = 62_400_000
        collect_turn(warmup=False)

    sweep = outputs['fft/sweep']
    assert all(msg.header == sweep.header for msg in outputs.values())
    assert sweep.header.stamp == rays[0].header.stamp and sweep.header.frame_id == 'navtech'
    assert sweep.end_stamp == rays[3].header.stamp
    assert list(sweep.timestamps) == [r.header.stamp for r in rays[:4]]
    bridge = CvBridge()
    assert sweep.image == outputs['fft/image']
    np.testing.assert_array_equal(bridge.imgmsg_to_cv2(sweep.image, 'mono16'), raw)

    scan = outputs['scan']
    np.testing.assert_array_equal(scan.ranges, bins * 0.125)
    np.testing.assert_array_equal(scan.intensities, amplitudes)
    assert scan.time_increment == 0.0 and scan.scan_time == 0.25
    projected = LaserProjection().projectLaser(scan, channel_options=LaserProjection.ChannelOption.INTENSITY)
    assert projected.header == sweep.header
    fields = ['x', 'y', 'z', 'intensity']
    directions = np.array([[1, 0], [0, -1], [-1, 0], [0, 1]])
    nearest = np.column_stack((directions * (bins * 0.125)[:, None], np.zeros(4), amplitudes))
    np.testing.assert_allclose(point_cloud2.read_points_numpy(projected, fields), nearest, atol=1e-5)
    extra = [[0, col * 0.125, 0, 30000] for col in range(352, 464, 4)]
    cloud = outputs['point_cloud']
    assert [f.name for f in cloud.fields] == fields and cloud.point_step == 16
    np.testing.assert_allclose(point_cloud2.read_points_numpy(cloud, fields), np.vstack((nearest, extra)),
                               atol=1e-5)

    polar = bridge.imgmsg_to_cv2(outputs['fft/display'], 'bgr8')
    plan = bridge.imgmsg_to_cv2(outputs['fft/cartesian'], 'bgr8')
    center = (plan.shape[0] - 1) / 2
    for row, (x, y) in enumerate(directions):
        # The polar view starts at 2 m (bin 16). On the plan, +x is up and +y left.
        color = polar[52 + row, 48 + bins[row] - 16]
        assert np.any(color)
        radius = (bins[row] + 1) * 0.125 * center / 64
        np.testing.assert_array_equal(plan[round(center - x * radius), round(center - y * radius)], color)
        assert not np.any(polar[52 + row, 48 + bins[row] - 20])
