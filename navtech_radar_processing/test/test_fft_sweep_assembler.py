"""Lossless assembly, stream boundary, resource limit and ROS contract tests."""

from array import array
from copy import deepcopy
import math
import time

from fft_sweep_assembler import FftSweepAssembler
from navtech_radar_msgs.msg import FftSpoke, FftSweep, RadarConfiguration
import numpy as np
import pytest
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from rclpy.serialization import deserialize_message, serialize_message


class Recorder:

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(deserialize_message(serialize_message(message), type(message)))


@pytest.fixture
def context():
    context = Context()
    context.init()
    yield context
    context.try_shutdown()


@pytest.fixture
def make_node(context):
    nodes = []

    def create(**parameters):
        # Retain the instance even when constructor validation raises.
        node = FftSweepAssembler.__new__(FftSweepAssembler)
        nodes.append(node)
        node.__init__(
            context=context,
            parameter_overrides=[Parameter(k, value=v) for k, v in parameters.items()],
        )
        node._sweep_publisher = Recorder()
        node._image_publisher = Recorder()
        node._point_cloud_publisher = Recorder()
        node._scan_publisher = Recorder()
        return node

    yield create
    for node in nodes:
        node.destroy_node()


def spoke(azimuth, *, angle=None, stamp=10, version=0, values=(0, 256, 65535),
          bits=16, frame='radar', counter=0):
    message = FftSpoke(
        azimuth=azimuth,
        azimuth_angle=math.radians(azimuth if angle is None else angle),
        configuration_version=version, sample_bit_depth=bits,
        data=array('H', values), sweep_counter=counter,
    )
    message.header.frame_id = frame
    message.header.stamp.sec = stamp
    message.host_stamp.sec = stamp + 1
    return message


def configuration(version=1, resolution=0.5, bins=3, azimuths=4):
    message = RadarConfiguration(
        configuration_version=version, range_resolution=resolution,
        azimuth_samples=azimuths, encoder_size=360,
        range_bins=bins, sample_bit_depth=16,
    )
    message.header.frame_id = 'radar'
    return message


def pixels(sweep):
    return np.frombuffer(sweep.image.data, dtype='<u2').reshape(
        sweep.image.height, sweep.image.width
    )


def test_wrap_preserves_uint16_and_aligned_metadata(make_node):
    node = make_node(expected_spokes=3)
    sources = [spoke(10, stamp=10), spoke(180, stamp=11), spoke(350, stamp=12)]
    for source in sources:
        node._spoke_callback(source)
    node._spoke_callback(spoke(0, stamp=13))
    sweep = node._sweep_publisher.messages[0]
    assert list(sweep.encoder_values) == [10, 180, 350]
    np.testing.assert_array_equal(pixels(sweep), [[0, 256, 65535]] * 3)
    assert sweep.image.encoding == 'mono16'
    assert not sweep.image.is_bigendian
    assert sweep.image.step == 6
    assert sweep.header.frame_id == 'radar'
    assert sweep.header.stamp.sec == 10
    assert sweep.end_stamp.sec == 12
    assert [t.sec for t in sweep.timestamps] == [10, 11, 12]
    assert [t.sec for t in sweep.host_stamps] == [11, 12, 13]
    assert len(node._spokes) == 1


def test_stable_encoder_order_retains_duplicate_rows(make_node):
    node = make_node()
    node._spoke_callback(spoke(20, angle=5, stamp=1, values=[1, 2]))
    node._spoke_callback(spoke(10, angle=10, stamp=2, values=[3, 4]))
    node._spoke_callback(spoke(10, angle=15, stamp=3, values=[5, 6]))
    node._finish_sweep()
    sweep = node._sweep_publisher.messages[0]
    assert list(sweep.encoder_values) == [10, 10, 20]
    assert [t.sec for t in sweep.timestamps] == [2, 3, 1]
    np.testing.assert_array_equal(pixels(sweep), [[3, 4], [5, 6], [1, 2]])


def test_payload_view_does_not_copy_or_modify_source(make_node):
    node = make_node()
    source = spoke(0)
    node._spoke_callback(source)
    assert np.shares_memory(node._spokes[0].values, np.asarray(source.data))
    assert not node._spokes[0].values.flags.writeable
    assert source.header.frame_id == 'radar'
    del source
    np.testing.assert_array_equal(node._spokes[0].values, [0, 256, 65535])


def test_configuration_transition_flushes_using_old_geometry(make_node):
    node = make_node(range_resolution=0.25)
    node._spoke_callback(spoke(0))
    node._spoke_callback(spoke(90))
    node._configuration_callback(configuration())
    old = node._sweep_publisher.messages[0]
    assert old.configuration_version == 0
    assert old.range_resolution == 0.25
    node._spoke_callback(spoke(180, version=1))
    node._spoke_callback(spoke(270, version=1))
    node._configuration_callback(configuration(version=2, resolution=1.0))
    old = node._sweep_publisher.messages[1]
    assert old.configuration_version == 1
    assert old.range_resolution == 0.5
    assert not node._spokes
    assert node._range_resolution == 1.0


def test_repeated_configuration_does_not_split_sweep(make_node):
    node = make_node()
    node._configuration_callback(configuration())
    node._spoke_callback(spoke(180, version=1))
    node._configuration_callback(configuration())
    assert len(node._spokes) == 1
    assert node._spokes[-1].angle == math.pi


def test_cross_topic_reordering_never_uses_wrong_configuration(make_node):
    node = make_node()
    node._spoke_callback(spoke(0, version=1))
    assert not node._spokes
    node._configuration_callback(configuration())
    node._spoke_callback(spoke(90, version=0))
    node._spoke_callback(spoke(180, version=2))
    node._spoke_callback(spoke(270, version=1))
    assert len(node._spokes) == 1
    assert node._invalid_spokes == 3


@pytest.mark.parametrize('change', ['width', 'bits', 'frame'])
def test_unconfigured_geometry_changes_never_mix_or_truncate(make_node, change):
    node = make_node()
    for azimuth in (0, 90):
        node._spoke_callback(spoke(azimuth, values=[1, 2, 3, 4]))
    options = {'values': [1, 2, 3, 4]}
    options.update({
        'width': {'values': [5, 6]}, 'bits': {'bits': 8}, 'frame': {'frame': 'other'},
    }[change])
    node._spoke_callback(spoke(180, **options))
    sweep = node._sweep_publisher.messages[0]
    assert sweep.image.width == 4
    np.testing.assert_array_equal(pixels(sweep), [[1, 2, 3, 4]] * 2)
    assert sweep.sample_bit_depth == 16
    assert sweep.header.frame_id == 'radar'
    assert len(node._spokes) == 1


@pytest.mark.parametrize('options', [
    {'values': []}, {'values': [1, 2, 3, 4]}, {'bits': 8}, {'azimuth': 360},
    {'frame': 'other'}, {'frame': ''}, {'frame': '/radar'}, {'angle': float('nan')},
])
def test_invalid_configured_spokes_are_rejected(make_node, options):
    node = make_node()
    node._configuration_callback(configuration())
    arguments = {'azimuth': 90, 'version': 1}
    arguments.update(options)
    node._spoke_callback(spoke(**arguments))
    assert not node._spokes
    assert node._invalid_spokes == 1


def test_frame_override_is_an_assertion_not_a_transform(make_node):
    node = make_node(output_frame='base_link')
    node._spoke_callback(spoke(0))
    assert not node._spokes
    node._spoke_callback(spoke(0, frame='base_link'))
    assert len(node._spokes) == 1


def test_invalid_8bit_payload_does_not_finish_pending_sweep(make_node):
    node = make_node()
    node._spoke_callback(spoke(180))
    node._spoke_callback(spoke(270))
    node._spoke_callback(spoke(0, bits=8, values=[0, 256, 0]))
    assert len(node._spokes) == 2
    assert node._spokes[-1].angle == 3 * math.pi / 2
    assert not node._sweep_publisher.messages
    assert node._invalid_spokes == 1


def test_invalid_configuration_retains_previous_geometry(make_node):
    node = make_node()
    node._configuration_callback(configuration())
    node._spoke_callback(spoke(90, version=1))
    node._configuration_callback(configuration(version=2, resolution=float('nan')))
    assert len(node._spokes) == 1
    assert node._configuration_version == 1
    assert node._range_resolution == 0.5


def test_backward_sensor_time_resets_partial_sweep(make_node):
    node = make_node()
    node._spoke_callback(spoke(0, stamp=100))
    node._spoke_callback(spoke(90, stamp=101))
    node._spoke_callback(spoke(180, stamp=50))
    assert len(node._spokes) == 1
    assert node._spokes[0].stamp.sec == 50
    assert node._dropped_sweeps == 1
    assert not node._sweep_publisher.messages


def test_no_wrap_stream_remains_bounded(make_node):
    node = make_node(expected_spokes=4, max_buffered_spokes=4, max_range_bins=4)
    for i in range(20):
        node._spoke_callback(spoke(90, stamp=i))
        assert len(node._spokes) <= 4
    assert node._dropped_sweeps == 4
    assert not node._sweep_publisher.messages
    node._spoke_callback(spoke(90, stamp=21, values=[1] * 5))
    assert node._invalid_spokes == 1


@pytest.mark.parametrize('azimuths,published', [
    ([0, 90, 180, 270], True), ([0, 90, 180], False),
    ([0, 90, 180, 270, 300], False), ([0, 90, 180, 180], False),
])
def test_complete_sweep_requires_exact_unique_count(make_node, azimuths, published):
    node = make_node(expected_spokes=4, publish_partial_sweeps=False)
    for azimuth in azimuths:
        node._spoke_callback(spoke(azimuth))
    node._finish_sweep()
    assert bool(node._sweep_publisher.messages) == published


def test_peak_selection_cloud_bytes_and_empty_cloud(make_node):
    node = make_node(
        publish_point_cloud=True, range_resolution=0.5, min_range=0.0,
        max_range=3.0, intensity_threshold=100, max_points_per_spoke=1,
    )
    values = [0, 150, 0, 200, 0]
    for azimuth in (0, 90):
        node._spoke_callback(spoke(azimuth, values=values))
    node._finish_sweep()
    cloud = node._point_cloud_publisher.messages[0]
    points = np.frombuffer(cloud.data, dtype='<f4').reshape(-1, 4)
    np.testing.assert_allclose(points, [[1.5, 0, 0, 200], [0, -1.5, 0, 200]], atol=1e-6)
    assert cloud.row_step == cloud.width * cloud.point_step == len(cloud.data)
    assert cloud.header == node._sweep_publisher.messages[0].header
    assert not cloud.is_bigendian
    for azimuth in (0, 90):
        node._spoke_callback(spoke(azimuth, values=[0] * 5))
    node._finish_sweep()
    cloud = node._point_cloud_publisher.messages[1]
    assert cloud.width == cloud.row_step == len(cloud.data) == 0
    assert cloud.point_step == 16
    assert len(cloud.fields) == 4


def test_cfar_adapts_to_clutter_and_preserves_source(make_node):
    node = make_node(
        peak_selection='cfar', cfar_training_cells=4, cfar_guard_cells=1,
        cfar_threshold_offset=500.0, intensity_threshold=0,
        min_peak_separation=0.0, min_range=0.0, max_range=100.0,
        range_resolution=0.5,
    )
    values = np.full(80, 1000, dtype=np.uint16)
    values[40:] = 20000
    values[15], values[20], values[60], values[65] = 3000, 1100, 22000, 20100
    original = values.copy()
    # A hard clutter boundary also exceeds a cell-averaging threshold. This
    # detector does not promise to distinguish that edge from a reflector.
    np.testing.assert_array_equal(node._extract_peak_bins(values), [15, 40, 60])
    np.testing.assert_array_equal(values, original)


def test_cfar_matches_scalar_windows_without_uint16_overflow(make_node):
    node = make_node(
        peak_selection='cfar', cfar_training_cells=5, cfar_guard_cells=2,
        cfar_threshold_offset=1000.0, intensity_threshold=0,
        min_peak_separation=0.0, min_range=0.0, max_range=100.0,
        range_resolution=0.5,
    )
    values = np.random.default_rng(12).integers(20000, 65536, 150, dtype=np.uint16)
    expected = []
    for i in range(7, len(values) - 7):
        noise = np.concatenate((values[i - 7:i - 2], values[i + 3:i + 8])).mean()
        if (values[i] > values[i - 1] and values[i] >= values[i + 1]
                and values[i] > noise + 1000):
            expected.append(i)
    np.testing.assert_array_equal(node._extract_peak_bins(values), expected)


def test_cfar_separation_selects_strongest_nearby_peak_and_ties_nearest(make_node):
    node = make_node(
        peak_selection='cfar', cfar_training_cells=2, cfar_guard_cells=4,
        cfar_threshold_offset=100.0, intensity_threshold=0,
        min_peak_separation=1.5, min_range=0.0, max_range=100.0,
        range_resolution=0.5,
    )
    values = np.full(60, 1000, dtype=np.uint16)
    values[10], values[12], values[30], values[32] = 3000, 5000, 4000, 4000
    np.testing.assert_array_equal(node._extract_peak_bins(values), [12, 30])
    # CFAR requires both complete training windows.
    assert node._extract_peak_bins(np.array([0, 65535, 0], dtype=np.uint16)).size == 0


@pytest.mark.parametrize('selection,expected', [('nearest', 0.5), ('strongest', 1.5)])
def test_scan_one_return_and_cloud_multiple_returns_share_geometry(make_node, selection, expected):
    node = make_node(
        publish_scan=True, publish_point_cloud=True, scan_selection=selection,
        range_resolution=0.5, expected_spokes=4, min_range=0.0,
        max_range=3.0, intensity_threshold=100, max_points_per_spoke=0,
        azimuth_offset_degrees=90.0,
    )
    sources = [spoke(a, values=[0, 150, 0, 200, 0]) for a in (0, 90, 180, 270)]
    before = deepcopy(sources)
    for source in sources:
        node._spoke_callback(source)
    node._finish_sweep()
    scan = node._scan_publisher.messages[0]
    cloud = node._point_cloud_publisher.messages[0]
    np.testing.assert_allclose(scan.ranges, [expected] * 4)
    assert scan.angle_min == pytest.approx(math.pi / 2)
    assert scan.angle_increment == pytest.approx(-math.pi / 2)
    assert scan.angle_max == pytest.approx(-math.pi)
    assert cloud.width == 8
    points = np.frombuffer(cloud.data, dtype='<f4').reshape(-1, 4)
    directions = np.array([[0, 1], [1, 0], [0, -1], [-1, 0]])
    np.testing.assert_allclose(points[::2, :2], 0.5 * directions, atol=1e-6)
    np.testing.assert_allclose(points[1::2, :2], 1.5 * directions, atol=1e-6)
    angles = scan.angle_min + np.arange(4) * scan.angle_increment
    np.testing.assert_allclose(np.column_stack((np.cos(angles), np.sin(angles))),
                               directions, atol=1e-6)
    assert scan.header == cloud.header
    assert sources == before


def test_scan_nearest_selection_precedes_cloud_budget(make_node):
    node = make_node(
        publish_scan=True, publish_point_cloud=True, expected_spokes=4,
        range_resolution=0.5, min_range=0.0, max_range=3.0,
        intensity_threshold=100, max_points_per_spoke=1,
    )
    for azimuth in (0, 90):
        node._spoke_callback(spoke(azimuth, values=[0, 150, 0, 200, 0]))
    node._finish_sweep()
    np.testing.assert_allclose(node._scan_publisher.messages[0].ranges[:2], [0.5, 0.5])
    cloud = node._point_cloud_publisher.messages[0]
    np.testing.assert_allclose(np.frombuffer(cloud.data, dtype='<f4').reshape(-1, 4)[:, 3], [200, 200])


def test_partial_scan_start_stamp_gaps_duplicates_and_no_return(make_node):
    node = make_node(
        publish_scan=True, expected_spokes=4, range_resolution=0.5,
        min_range=0.0, max_range=3.0, intensity_threshold=100,
    )
    for azimuth, values in ((90, [0, 150, 0, 0, 0]),
                            (90, [0, 0, 0, 200, 0]),
                            (270, [0] * 5)):
        node._spoke_callback(spoke(azimuth, values=values, stamp=12))
    node._finish_sweep()
    scan = node._scan_publisher.messages[0]
    assert scan.angle_min == pytest.approx(3 * math.pi / 2)
    assert scan.header.stamp.sec == 12
    assert scan.ranges[0] == 0.5
    assert math.isnan(scan.ranges[1]) and math.isnan(scan.ranges[3])
    assert math.isnan(scan.ranges[2])
    assert list(scan.intensities) == [150, 0, 0, 0]
    assert scan.time_increment == 0.0
    assert scan.range_max == 2.0


def test_scan_timing_uses_sensor_timestamps_and_resets_on_irregularity(make_node):
    node = make_node(publish_scan=True, min_range=0.0, max_range=10.0)
    config = configuration(bins=5)
    config.expected_rotation_rate = 4
    node._configuration_callback(config)
    for sweep_number in range(2):
        for index, azimuth in enumerate((0, 90, 180, 270)):
            message = spoke(azimuth, stamp=10 + sweep_number, version=1, values=[0, 30000, 0, 0, 0])
            message.header.stamp.nanosec = index * 62500000
            if sweep_number and index == 2:
                message.header.stamp.nanosec += 20000000
            node._spoke_callback(message)
        node._finish_sweep()
    first, second = node._scan_publisher.messages
    assert first.scan_time == second.scan_time == 0.25
    assert first.time_increment == 0.0625
    assert second.time_increment == 0.0
    node._spoke_callback(spoke(0, stamp=1, version=1, values=[0] * 5))
    node._spoke_callback(spoke(90, stamp=1, version=1, values=[0] * 5))
    node._finish_sweep()
    assert node._scan_publisher.messages[-1].header.stamp.sec == 1
    assert node._scan_publisher.messages[-1].time_increment == 0.0


def test_off_grid_rays_do_not_fill_missing_bearings(make_node):
    node = make_node(publish_scan=True, expected_spokes=4, min_range=0.0)
    for azimuth in (0, 130, 180):
        node._spoke_callback(spoke(azimuth))
    node._finish_sweep()
    assert math.isnan(node._scan_publisher.messages[0].ranges[1])


@pytest.mark.parametrize('parameters', [
    {'range_resolution': 0.0}, {'min_range': float('nan')},
    {'max_range': float('inf')}, {'min_range': 4.0, 'max_range': 2.0},
    {'input_queue_depth': 0}, {'minimum_spokes': 0}, {'expected_spokes': 1},
    {'max_buffered_spokes': 399}, {'max_range_bins': 0},
    {'intensity_threshold': -1}, {'max_points_per_spoke': -1},
    {'wrap_threshold_degrees': 360.0}, {'output_frame': '/radar'},
    {'peak_selection': 'random'}, {'scan_selection': 'all'},
    {'cfar_training_cells': 0}, {'cfar_guard_cells': -1},
    {'cfar_threshold_offset': float('nan')}, {'min_peak_separation': -1.0},
])
def test_invalid_startup_parameters_fail_early(make_node, parameters):
    with pytest.raises(ValueError):
        make_node(**parameters)


def test_namespaced_dds_pipeline(context):
    node = FftSweepAssembler(context=context, namespace='/robot')
    peer = Node('peer', context=context, namespace='/robot')
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    executor.add_node(peer)
    config_pub = peer.create_publisher(
        RadarConfiguration, 'navtech_radar/configuration',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    spoke_pub = peer.create_publisher(
        FftSpoke, 'navtech_radar/fft/spoke', qos_profile_sensor_data
    )
    received = []
    peer.create_subscription(FftSweep, 'navtech_radar/fft/sweep', received.append, 2)

    def spin_until(condition):
        deadline = time.monotonic() + 5.0
        while not condition() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert condition(), 'ROS delivery timed out'

    try:
        assert node._image_publisher.topic_name == '/robot/navtech_radar/fft/image'
        assert node._point_cloud_publisher.topic_name == '/robot/navtech_radar/point_cloud'
        assert node._scan_publisher.topic_name == '/robot/navtech_radar/scan'
        result = node.set_parameters([Parameter('intensity_threshold', value=1)])
        assert not result[0].successful
        spin_until(lambda: spoke_pub.get_subscription_count() == 1)
        spin_until(lambda: node._sweep_publisher.get_subscription_count() == 1)
        config_pub.publish(configuration())
        spin_until(lambda: node._configuration_version == 1)
        for count, azimuth in enumerate((0, 90, 180, 270), 1):
            spoke_pub.publish(spoke(azimuth, version=1))
            spin_until(lambda: len(node._spokes) == count)
        spoke_pub.publish(spoke(0, version=1))
        spin_until(lambda: bool(received))
        assert list(received[0].encoder_values) == [0, 90, 180, 270]
        np.testing.assert_array_equal(pixels(received[0]), [[0, 256, 65535]] * 4)
    finally:
        executor.shutdown()
        peer.destroy_node()
        node.destroy_node()


@pytest.mark.parametrize('gain,offset', [(-1.0, 0.0), (0.0, 1.0), (float('nan'), 0.0),
                                         (1.0, float('inf'))])
def test_invalid_range_calibration_retains_active_config(make_node, gain, offset):
    node = make_node()
    node._configuration_callback(configuration())
    cfg = configuration(version=2)
    cfg.range_gain, cfg.range_offset = gain, offset
    node._configuration_callback(cfg)
    assert node._configuration_version == 1


def test_cfar_separation_uses_calibrated_metres(make_node):
    node = make_node(peak_selection='cfar', cfar_training_cells=1, cfar_guard_cells=0,
                     cfar_threshold_offset=0.0, intensity_threshold=100,
                     min_peak_separation=1.5, min_range=1.0, max_range=3.0)
    cfg = configuration(bins=8)
    cfg.range_gain, cfg.range_offset = 2.0, -1.0
    node._configuration_callback(cfg)
    levels = np.array([0, 0, 300, 0, 500, 0, 700, 0], dtype=np.uint16)
    np.testing.assert_array_equal(node._extract_peak_bins(levels), [2, 4])


def test_duplicate_empty_ray_invalidates_uniform_timing(make_node):
    node = make_node(publish_scan=True, min_range=0.0)
    cfg = configuration()
    cfg.expected_rotation_rate = 4
    node._configuration_callback(cfg)
    for index in (0, 0, 1, 2, 3):
        msg = spoke(index * 90, version=1, values=[0] * 3)
        msg.header.stamp.nanosec = index * 62500000
        node._spoke_callback(msg)
    node._finish_sweep()
    scan = node._scan_publisher.messages[0]
    assert all(math.isnan(r) for r in scan.ranges)
    assert scan.time_increment == 0.0


def test_gap_at_one_revolution_starts_new_partial_sweep(make_node):
    node = make_node()
    cfg = configuration()
    cfg.expected_rotation_rate = 4
    node._configuration_callback(cfg)
    for az, ns in [(0, 0), (90, 62500000), (180, 312500000), (270, 375000000), (0, 437500000)]:
        msg = spoke(az, version=1)
        msg.header.stamp.nanosec = ns
        node._spoke_callback(msg)
    assert [list(s.encoder_values) for s in node._sweep_publisher.messages] == [[180, 270]]
    assert node._sweep_publisher.messages[0].header.stamp.nanosec == 312500000


def test_encoder_validation_is_circular_and_precedes_state_changes(make_node):
    node = make_node()
    node._configuration_callback(configuration())
    node._spoke_callback(spoke(0, angle=359.8, version=1))
    node._spoke_callback(spoke(90, angle=180, version=1, stamp=5))
    node._spoke_callback(spoke(90, angle=90.4, version=1))
    assert node._invalid_spokes == 1
    assert [s.angle for s in node._spokes] == [0.0, math.pi / 2]


@pytest.mark.parametrize('angles,accepted', [
    ([0, 1, 180, 270], False), ([0, 120, 180, 270], False),
    ([45, 135, 225, 315], True), ([0, 90, 180, 270], True),
])
def test_complete_requires_distinct_image_and_scan_cells(make_node, angles, accepted):
    node = make_node(publish_partial_sweeps=False)
    node._configuration_callback(configuration())
    for a in angles:
        node._spoke_callback(spoke(a, version=1))
    node._finish_sweep()
    assert bool(node._sweep_publisher.messages) == accepted


def test_unknown_rate_uses_fallback_gap(make_node):
    node = make_node(fallback_max_spoke_gap_sec=2.0)
    for az, sec in [(0, 1), (90, 3), (180, 12), (270, 13), (0, 14)]:
        node._spoke_callback(spoke(az, stamp=sec))
    assert [list(s.encoder_values) for s in node._sweep_publisher.messages] == [[180, 270]]


@pytest.mark.parametrize('encoder,advance,gap_ns,boundary', [
    (90, 3, 187500000, True),     # 180 -> 90 across zero, three packets onward
    (180, 4, 249000000, True),    # same bearing one slightly fast turn later
    (90, 0, 187500000, False),    # absent or repeated counter
    (90, 1, 187500000, False),    # position and counter disagree
    (90, 3, 1000000, False),      # elapsed sensor time contradicts a wrap
    (45, 3, 156250000, False),    # exactly half a cell from the predicted bearing
    (225, 1, 62500000, False),    # ordinary forward motion
    (180, 0, 0, False),           # duplicate spoke stays in the same sweep
])
def test_counter_evidence_for_wrap(make_node, encoder, advance, gap_ns, boundary):
    node = make_node()
    cfg = configuration()
    cfg.expected_rotation_rate = 4
    node._configuration_callback(cfg)
    node._spoke_callback(spoke(90, counter=65533, version=1))
    node._spoke_callback(spoke(180, counter=65534, version=1))
    source = spoke(encoder, counter=(65534 + advance) % 65536, version=1)
    source.header.stamp.nanosec = gap_ns
    node._spoke_callback(source)
    assert bool(node._sweep_publisher.messages) == boundary
    assert len(node._spokes) == (1 if boundary else 3)


def test_hidden_wrap_closes_a_complete_sweep_in_strict_mode(make_node):
    node = make_node(publish_partial_sweeps=False)
    node._configuration_callback(configuration())
    # Dropping the first two rays of turn two leaves only a 90-degree
    # backwards jump. Strict mode should still publish the complete first turn.
    for tick in (0, 1, 2, 3, 6, 7, 8):
        node._spoke_callback(spoke(90 * (tick % 4), counter=tick, version=1))
    assert len(node._sweep_publisher.messages) == 1
    assert list(node._sweep_publisher.messages[0].sweep_counters) == [0, 1, 2, 3]
