"""Regression tests using the actual ROS interfaces and node configuration."""

from array import array
import math
import time

from legacy_bag_driver import LegacyBagDriver
from nav_ross.msg import HighPrecisionFFTData, nav_msg
from navtech_radar_msgs.msg import RadarConfiguration
import pytest
from rclpy.clock import ClockType
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
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
def driver(context):
    node = LegacyBagDriver(context=context, namespace='/robot')
    node._configuration_publisher = Recorder()
    node._spoke_publisher = Recorder()
    node._diagnostics_publisher = Recorder()
    yield node
    node.destroy_node()


def configuration():
    return nav_msg(
        range_resolution=0.0438, AzimuthSamples=400, EncoderSize=5600,
        BinSize=438, RangeInBins=4, ExpectedRotationRate=4,
    )


def spoke():
    message = HighPrecisionFFTData(
        angle=90.0, azimuth=1400, sweepCounter=65535, size=4,
        data=array('H', [0, 255, 256, 65535]),
    )
    message.header.frame_id = 'legacy_frame'
    message.header.stamp.sec = 1713882442
    message.header.stamp.nanosec = 526653863
    return message


def test_lossless_conversion_without_mutating_input(driver):
    driver._configuration_callback(configuration())
    source = spoke()
    driver._fft_callback(source)
    result = driver._spoke_publisher.messages[0]
    assert list(result.data) == [0, 255, 256, 65535]
    assert result.header.stamp == source.header.stamp
    assert result.header.frame_id == 'navtech'
    assert source.header.frame_id == 'legacy_frame'
    assert result.configuration_version == 1
    assert result.sample_bit_depth == 16
    assert result.sweep_counter == 65535
    assert result.azimuth == 1400
    assert result.azimuth_angle == pytest.approx(math.pi / 2)
    assert result.ntp_seconds == 1713882442
    assert result.ntp_split_seconds == 526653863


def test_config_deduplication_and_versioning(driver):
    message = configuration()
    driver._configuration_callback(message)
    driver._configuration_callback(message)
    assert len(driver._configuration_publisher.messages) == 1
    assert driver._configuration_publisher.messages[0].packet_rate == 1600
    message.RangeInBins = 5
    driver._configuration_callback(message)
    assert driver._configuration_publisher.messages[-1].configuration_version == 2


@pytest.mark.parametrize('field,value', [
    ('range_resolution', float('nan')), ('range_resolution', 0.0),
    ('AzimuthSamples', 0), ('EncoderSize', 1), ('RangeInBins', 0),
    ('ExpectedRotationRate', 65535),
])
def test_invalid_configuration_does_not_poison_state(driver, field, value):
    message = configuration()
    setattr(message, field, value)
    driver._configuration_callback(message)
    assert driver._configuration_version == 0
    assert driver._configuration_key is None
    assert driver._invalid_configurations == 1
    driver._configuration_callback(configuration())
    assert driver._configuration_version == 1


@pytest.mark.parametrize('kind', ['size', 'angle', 'empty', 'time', 'bins', 'encoder'])
def test_malformed_spokes_are_rejected(driver, kind):
    driver._configuration_callback(configuration())
    message = spoke()
    if kind == 'size':
        message.size = 5
    elif kind == 'angle':
        message.angle = float('inf')
    elif kind == 'empty':
        message.data = []
    elif kind == 'time':
        message.header.stamp.nanosec = 1_000_000_000
    elif kind == 'bins':
        message.data.append(1)
        message.size = 5
    else:
        message.azimuth = 5600
    driver._fft_callback(message)
    assert not driver._spoke_publisher.messages
    assert driver._invalid_spokes == 1


def test_angle_normalization_and_unconfigured_spoke(driver):
    message = spoke()
    message.angle = -90.0
    driver._fft_callback(message)
    result = driver._spoke_publisher.messages[0]
    assert result.azimuth_angle == pytest.approx(3 * math.pi / 2)
    assert result.configuration_version == 0


def spin_until(executor, condition, timeout=5.0):
    deadline = time.monotonic() + timeout
    while not condition() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    assert condition(), 'ROS delivery timed out'


def test_namespace_qos_and_late_configuration_subscriber(context):
    node = LegacyBagDriver(context=context, namespace='/robot')
    observer = Node('observer', context=context, namespace='/robot')
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    executor.add_node(observer)
    try:
        assert node._spoke_publisher.topic_name == '/robot/navtech_radar/fft/spoke'
        assert node._diagnostics_publisher.topic_name == '/robot/diagnostics'
        assert node._fft_subscription.topic_name == '/Navtech/FFTData'
        assert node._spoke_publisher.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
        assert node._diagnostic_timer.clock.clock_type == ClockType.STEADY_TIME
        result = node.set_parameters([Parameter('frame_id', value='another_frame')])
        assert not result[0].successful
        node._configuration_callback(configuration())
        received = []
        observer.create_subscription(
            RadarConfiguration, 'navtech_radar/configuration', received.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        spin_until(executor, lambda: bool(received))
        assert received[0].range_bins == 4
    finally:
        executor.shutdown()
        observer.destroy_node()
        node.destroy_node()


def test_diagnostics_run_without_simulated_clock(context):
    node = LegacyBagDriver(
        context=context, parameter_overrides=[Parameter('use_sim_time', value=True)]
    )
    node._diagnostics_publisher = Recorder()
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    try:
        spin_until(executor, lambda: bool(node._diagnostics_publisher.messages), timeout=3.0)
        status = node._diagnostics_publisher.messages[-1].status[0]
        assert status.level == status.WARN
    finally:
        executor.shutdown()
        node.destroy_node()
