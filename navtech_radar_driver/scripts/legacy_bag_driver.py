#!/usr/bin/env python3
"""Adapt the dataset's legacy Navtech topics to the lossless driver API."""

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_ross.msg import HighPrecisionFFTData, nav_msg
from navtech_radar_msgs.msg import FftSpoke, RadarConfiguration
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


class LegacyBagDriver(Node):
    """Present legacy ROS 1-derived bag messages as driver output."""

    def __init__(self, **kwargs) -> None:
        super().__init__('navtech_legacy_bag_driver', **kwargs)

        defaults = {
            'legacy_configuration_topic': '/Navtech/Configuration_Topic',
            'legacy_fft_topic': '/Navtech/FFTData',
            'configuration_topic': 'navtech_radar/configuration',
            'spoke_topic': 'navtech_radar/fft/spoke',
            'frame_id': 'navtech',
            'input_queue_depth': 512,
            'output_queue_depth': 512,
        }
        for name, value in defaults.items():
            self.declare_parameter(
                name, value, ParameterDescriptor(read_only=True)
            )

        self._frame_id = str(self.get_parameter('frame_id').value)
        if not self._frame_id or self._frame_id.startswith('/'):
            raise ValueError('frame_id must be nonempty and have no leading slash')
        for name in ('input_queue_depth', 'output_queue_depth'):
            if self.get_parameter(name).value <= 0:
                raise ValueError(f'{name} must be positive')
        self._configuration_version = 0
        self._configuration_key = None
        self._configuration_messages = 0
        self._invalid_configurations = 0
        self._spokes = 0
        self._invalid_spokes = 0
        self._last_spoke_wall_time = None
        self._last_report_wall_time = time.monotonic()
        self._last_report_spokes = 0

        configuration_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        spoke_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=int(self.get_parameter('output_queue_depth').value),
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        legacy_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=int(self.get_parameter('input_queue_depth').value),
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._configuration_publisher = self.create_publisher(
            RadarConfiguration,
            str(self.get_parameter('configuration_topic').value),
            configuration_qos,
        )
        self._spoke_publisher = self.create_publisher(
            FftSpoke,
            str(self.get_parameter('spoke_topic').value),
            spoke_qos,
        )
        self._diagnostics_publisher = self.create_publisher(
            DiagnosticArray, 'diagnostics', 10
        )

        self._configuration_subscription = self.create_subscription(
            nav_msg,
            str(self.get_parameter('legacy_configuration_topic').value),
            self._configuration_callback,
            legacy_qos,
        )
        self._fft_subscription = self.create_subscription(
            HighPrecisionFFTData,
            str(self.get_parameter('legacy_fft_topic').value),
            self._fft_callback,
            legacy_qos,
        )
        # Health reporting must continue when simulated time is paused or absent.
        self._diagnostic_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._diagnostic_timer = self.create_timer(
            1.0, self._publish_diagnostics, clock=self._diagnostic_clock
        )

        self.get_logger().info(
            'Waiting for legacy high-precision Navtech FFT bag topics'
        )

    def _configuration_callback(self, message: nav_msg) -> None:
        self._configuration_messages += 1
        configuration_key = (
            float(message.range_resolution),
            int(message.AzimuthSamples),
            int(message.EncoderSize),
            int(message.BinSize),
            int(message.RangeInBins),
            int(message.ExpectedRotationRate),
        )
        resolution, azimuths, encoder_size, bin_size, bins, rate = configuration_key
        if (
            not math.isfinite(resolution) or resolution <= 0.0
            or not 0 < azimuths <= encoder_size
            or bin_size == 0 or bins == 0 or rate == 0
            or azimuths * rate > 65535
        ):
            self._invalid_configurations += 1
            self.get_logger().warning(
                'Ignoring invalid legacy radar configuration',
                throttle_duration_sec=5.0,
            )
            return
        if configuration_key == self._configuration_key:
            return

        self._configuration_key = configuration_key
        self._configuration_version += 1

        output = RadarConfiguration()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = self._frame_id
        output.configuration_version = self._configuration_version
        output.range_resolution = float(message.range_resolution)
        output.azimuth_samples = int(message.AzimuthSamples)
        output.encoder_size = int(message.EncoderSize)
        output.bin_size = int(message.BinSize)
        output.range_bins = int(message.RangeInBins)
        output.expected_rotation_rate = int(message.ExpectedRotationRate)
        output.packet_rate = output.azimuth_samples * output.expected_rotation_rate
        output.range_gain = 1.0
        output.range_offset = 0.0
        output.sample_bit_depth = 16
        self._configuration_publisher.publish(output)

        self.get_logger().info(
            'Configuration version '
            f'{output.configuration_version}: '
            f'{output.azimuth_samples} azimuths, '
            f'{output.range_bins} bins, '
            f'{output.range_resolution:.6f} m/bin, '
            f'{output.expected_rotation_rate} Hz'
        )

    def _fft_callback(self, message: HighPrecisionFFTData) -> None:
        angle_degrees = float(message.angle)
        if (
            not math.isfinite(angle_degrees) or not message.data
            or len(message.data) > 65535
            or message.header.stamp.sec < 0
            or message.header.stamp.nanosec >= 1_000_000_000
        ):
            self._invalid_spokes += 1
            return

        if int(message.size) != len(message.data):
            self._invalid_spokes += 1
            self.get_logger().warning(
                'Legacy FFT size field does not match payload: '
                f'{message.size} != {len(message.data)}',
                throttle_duration_sec=5.0,
            )
            return

        if self._configuration_key is not None:
            if (
                len(message.data) != self._configuration_key[4]
                or message.azimuth >= self._configuration_key[2]
            ):
                self._invalid_spokes += 1
                return

        output = FftSpoke()
        output.header.stamp = message.header.stamp
        output.header.frame_id = self._frame_id
        output.host_stamp = self.get_clock().now().to_msg()
        output.configuration_version = self._configuration_version
        output.ntp_seconds = int(message.header.stamp.sec)
        output.ntp_split_seconds = int(message.header.stamp.nanosec)
        output.sweep_counter = int(message.sweepCounter)
        output.azimuth = int(message.azimuth)
        output.azimuth_angle = math.radians(angle_degrees % 360.0)
        output.sample_bit_depth = 16
        output.data = message.data
        self._spoke_publisher.publish(output)

        self._spokes += 1
        self._last_spoke_wall_time = time.monotonic()

    def _publish_diagnostics(self) -> None:
        now = time.monotonic()
        elapsed = max(now - self._last_report_wall_time, 1e-9)
        spoke_rate = (self._spokes - self._last_report_spokes) / elapsed
        receiving = (
            self._last_spoke_wall_time is not None
            and now - self._last_spoke_wall_time < 2.0
        )

        status = DiagnosticStatus()
        status.name = f'{self.get_fully_qualified_name()}: legacy playback transport'
        status.hardware_id = 'navtech-bag-playback'
        status.level = DiagnosticStatus.OK if receiving else DiagnosticStatus.WARN
        status.message = (
            'Receiving high-precision FFT spokes'
            if receiving
            else 'Waiting for high-precision FFT spokes'
        )
        status.values = [
            KeyValue(key='configuration_version', value=str(
                self._configuration_version
            )),
            KeyValue(key='configuration_messages', value=str(
                self._configuration_messages
            )),
            KeyValue(key='invalid_configurations', value=str(
                self._invalid_configurations
            )),
            KeyValue(key='spokes', value=str(self._spokes)),
            KeyValue(key='spoke_rate_hz', value=f'{spoke_rate:.1f}'),
            KeyValue(key='invalid_spokes', value=str(self._invalid_spokes)),
            KeyValue(key='sample_bit_depth', value='16'),
        ]

        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        diagnostics.status = [status]
        self._diagnostics_publisher.publish(diagnostics)

        self._last_report_wall_time = now
        self._last_report_spokes = self._spokes


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = LegacyBagDriver()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
