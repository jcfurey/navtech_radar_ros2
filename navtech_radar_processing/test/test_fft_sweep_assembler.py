"""Lossless assembly, stream boundary, resource limit and ROS contract tests."""

from array import array
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
    assert node._last_angle is None
    assert node._range_resolution == 1.0


def test_repeated_configuration_does_not_split_sweep(make_node):
    node = make_node()
    node._configuration_callback(configuration())
    node._spoke_callback(spoke(180, version=1))
    node._configuration_callback(configuration())
    assert len(node._spokes) == 1
    assert node._last_angle == math.pi


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
    assert node._last_angle == 3 * math.pi / 2
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
    np.testing.assert_allclose(points, [[1.5, 0, 0, 200], [0, 1.5, 0, 200]], atol=1e-6)
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


@pytest.mark.parametrize('parameters', [
    {'range_resolution': 0.0}, {'min_range': float('nan')},
    {'max_range': float('inf')}, {'min_range': 4.0, 'max_range': 2.0},
    {'input_queue_depth': 0}, {'minimum_spokes': 0}, {'expected_spokes': 1},
    {'max_buffered_spokes': 399}, {'max_range_bins': 0},
    {'intensity_threshold': -1}, {'max_points_per_spoke': -1},
    {'wrap_threshold_degrees': 360.0}, {'output_frame': '/radar'},
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
