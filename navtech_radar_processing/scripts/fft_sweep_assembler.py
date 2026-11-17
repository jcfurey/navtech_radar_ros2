#!/usr/bin/env python3
"""Assemble lossless Navtech FFT spokes into metadata-rich revolutions."""

from array import array
from dataclasses import dataclass
import math

from cloud_deskew import CloudDeskew
from navtech_radar_msgs.msg import FftSpoke, FftSweep, RadarConfiguration
import numpy as np
from radar_geometry import encoder_cell, RangeAxis, ros_bearing, scan_cell
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan, PointCloud2, PointField


@dataclass
class StoredSpoke:
    angle: float
    azimuth: int
    sweep_counter: int
    configuration_version: int
    sample_bit_depth: int
    stamp: object
    host_stamp: object
    frame_id: str
    values: np.ndarray


class FftSweepAssembler(Node):
    """Create sweep and visualization products from canonical FFT spokes."""

    def __init__(self, **kwargs) -> None:
        super().__init__('navtech_fft_sweep_assembler', **kwargs)

        defaults = {
            'configuration_topic': 'navtech_radar/configuration',
            'spoke_topic': 'navtech_radar/fft/spoke',
            'sweep_topic': 'navtech_radar/fft/sweep',
            'image_topic': 'navtech_radar/fft/image',
            'point_cloud_topic': 'navtech_radar/point_cloud',
            'scan_topic': 'navtech_radar/scan',
            'output_frame': '',
            'deskew_fixed_frame': '',
            'range_resolution': 0.0438,
            'expected_spokes': 400,
            'fallback_max_spoke_gap_sec': 1.0,
            'publish_partial_sweeps': True,
            'minimum_spokes': 2,
            'publish_point_cloud': False,
            'publish_scan': False,
            'scan_selection': 'nearest',
            'peak_selection': 'threshold',
            'cfar_training_cells': 16,
            'cfar_guard_cells': 4,
            'cfar_threshold_offset': 3000.0,
            'min_peak_separation': 0.5,
            'min_range': 2.0,
            'max_range': 200.0,
            'intensity_threshold': 22000,
            'max_points_per_spoke': 48,
            'azimuth_offset_degrees': 0.0,
            'wrap_threshold_degrees': 180.0,
            'input_queue_depth': 512,
            'max_buffered_spokes': 2048,
            'max_range_bins': 16384,
        }
        for name, value in defaults.items():
            self.declare_parameter(
                name, value, ParameterDescriptor(read_only=True)
            )

        self._output_frame = str(self.get_parameter('output_frame').value)
        self._range_resolution = float(
            self.get_parameter('range_resolution').value
        )
        self._fallback_gap = self.get_parameter('fallback_max_spoke_gap_sec').value
        self._range_axis = RangeAxis(self._range_resolution, 0.0)
        self._expected_spokes = int(self.get_parameter('expected_spokes').value)
        self._publish_partial = bool(
            self.get_parameter('publish_partial_sweeps').value
        )
        self._minimum_spokes = int(self.get_parameter('minimum_spokes').value)
        self._publish_point_cloud = bool(
            self.get_parameter('publish_point_cloud').value
        )
        self._publish_scan = self.get_parameter('publish_scan').value
        self._scan_selection = self.get_parameter('scan_selection').value
        self._peak_selection = self.get_parameter('peak_selection').value
        self._cfar_training = self.get_parameter('cfar_training_cells').value
        self._cfar_guard = self.get_parameter('cfar_guard_cells').value
        self._cfar_offset = self.get_parameter('cfar_threshold_offset').value
        self._peak_separation = self.get_parameter('min_peak_separation').value
        self._min_range = float(self.get_parameter('min_range').value)
        self._max_range = float(self.get_parameter('max_range').value)
        self._threshold = int(
            self.get_parameter('intensity_threshold').value
        )
        self._max_points = int(
            self.get_parameter('max_points_per_spoke').value
        )
        self._azimuth_offset = math.radians(
            float(self.get_parameter('azimuth_offset_degrees').value)
        )
        self._wrap_threshold = math.radians(
            float(self.get_parameter('wrap_threshold_degrees').value)
        )
        self._max_buffered_spokes = int(
            self.get_parameter('max_buffered_spokes').value
        )
        self._max_range_bins = int(self.get_parameter('max_range_bins').value)
        self._validate_parameters()

        self._configuration_version = 0
        self._configuration_key = None
        self._range_bins = None
        self._encoder_size = None
        self._sample_bit_depth = None
        self._configuration_frame = None
        self._rotation_rate = 0
        self._last_angle = None
        self._spokes: list[StoredSpoke] = []
        self._sweep_sequence = 0
        self._dropped_sweeps = 0
        self._invalid_spokes = 0

        configuration_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        spoke_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=int(self.get_parameter('input_queue_depth').value),
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._sweep_publisher = self.create_publisher(
            FftSweep,
            str(self.get_parameter('sweep_topic').value),
            reliable_output_qos,
        )
        self._image_publisher = self.create_publisher(
            Image,
            str(self.get_parameter('image_topic').value),
            reliable_output_qos,
        )
        self._point_cloud_publisher = self.create_publisher(
            PointCloud2,
            str(self.get_parameter('point_cloud_topic').value),
            qos_profile_sensor_data,
        )
        fixed = self.get_parameter('deskew_fixed_frame').value
        if fixed.startswith('/'):
            raise ValueError('deskew_fixed_frame must have no leading slash')
        self._deskew = (CloudDeskew(self, fixed, self._publish_cloud_message)
                        if self._publish_point_cloud and fixed else None)
        self._scan_publisher = self.create_publisher(
            LaserScan, self.get_parameter('scan_topic').value, qos_profile_sensor_data,
        )
        self._configuration_subscription = self.create_subscription(
            RadarConfiguration,
            str(self.get_parameter('configuration_topic').value),
            self._configuration_callback,
            configuration_qos,
        )
        self._spoke_subscription = self.create_subscription(
            FftSpoke,
            str(self.get_parameter('spoke_topic').value),
            self._spoke_callback,
            spoke_qos,
        )

    def _validate_parameters(self) -> None:
        for name in (
            'range_resolution', 'min_range', 'max_range',
            'azimuth_offset_degrees', 'wrap_threshold_degrees',
            'cfar_threshold_offset', 'min_peak_separation',
        ):
            if not math.isfinite(self.get_parameter(name).value):
                raise ValueError(f'{name} must be finite')
        if not math.isfinite(self._fallback_gap) or self._fallback_gap <= 0:
            raise ValueError('fallback_max_spoke_gap_sec must be finite and positive')
        if self._range_resolution <= 0:
            raise ValueError('range_resolution must be positive')
        if not 0 <= self._min_range < self._max_range:
            raise ValueError('ranges must satisfy 0 <= min_range < max_range')
        if not 0 <= self._threshold <= 65535 or self._max_points < 0:
            raise ValueError('invalid intensity_threshold or max_points_per_spoke')
        if self._peak_selection not in ('threshold', 'cfar'):
            raise ValueError('peak_selection must be threshold or cfar')
        if self._scan_selection not in ('nearest', 'strongest'):
            raise ValueError('scan_selection must be nearest or strongest')
        if not 1 <= self._cfar_training <= 1024 or not 0 <= self._cfar_guard <= 1024:
            raise ValueError('CFAR cells per side must be training [1,1024], guard [0,1024]')
        if self._cfar_offset < 0 or self._peak_separation < 0:
            raise ValueError('CFAR offset and peak separation must be nonnegative')
        if not 0 < self._wrap_threshold < 2 * math.pi:
            raise ValueError('wrap_threshold_degrees must be between 0 and 360')
        if not 1 <= self._minimum_spokes <= self._expected_spokes <= 65535:
            raise ValueError('require 1 <= minimum_spokes <= expected_spokes <= 65535')
        if self._max_buffered_spokes < self._expected_spokes:
            raise ValueError('max_buffered_spokes must be at least expected_spokes')
        if not 1 <= self._max_range_bins <= 65535:
            raise ValueError('max_range_bins must be between 1 and 65535')
        if self.get_parameter('input_queue_depth').value <= 0:
            raise ValueError('input_queue_depth must be positive')
        if self._output_frame.startswith('/'):
            raise ValueError('output_frame must have no leading slash')

    def _configuration_callback(self, message: RadarConfiguration) -> None:
        axis = RangeAxis.from_config(message)
        if (
            not axis.valid(message.range_bins)
            or not math.isfinite(message.range_resolution)
            or message.range_resolution <= 0
            or not 0 < message.azimuth_samples <= message.encoder_size
            or message.azimuth_samples > self._max_buffered_spokes
            or not 0 < message.range_bins <= self._max_range_bins
            or message.sample_bit_depth not in (8, 16)
            or message.configuration_version == 0
            or not message.header.frame_id
            or message.header.frame_id.startswith('/')
            or (self._output_frame and message.header.frame_id != self._output_frame)
        ):
            self.get_logger().warning(
                'Ignoring invalid or incompatible radar configuration',
                throttle_duration_sec=5.0,
            )
            return
        key = (
            message.configuration_version, message.range_resolution,
            message.azimuth_samples, message.encoder_size, message.range_bins,
            message.sample_bit_depth, message.header.frame_id,
            message.expected_rotation_rate, message.range_gain, message.range_offset,
        )
        if key == self._configuration_key:
            return
        # Flush with the old geometry, including the version-zero fallback.
        self._finish_sweep()
        self._configuration_key = key
        self._range_axis = axis

        self._configuration_version = int(message.configuration_version)
        self._range_resolution = float(message.range_resolution)
        self._expected_spokes = int(message.azimuth_samples)
        self._range_bins = int(message.range_bins)
        self._encoder_size = int(message.encoder_size)
        self._sample_bit_depth = int(message.sample_bit_depth)
        self._configuration_frame = message.header.frame_id
        self._rotation_rate = message.expected_rotation_rate

        self.get_logger().info(
            f'Using configuration version {self._configuration_version}: '
            f'{self._expected_spokes} spokes, {message.range_bins} bins, '
            f'{self._range_resolution:.6f} m/bin'
        )

    def _spoke_callback(self, message: FftSpoke) -> None:
        angle = float(message.azimuth_angle)
        if (
            not math.isfinite(angle)
            or not 0 < len(message.data) <= self._max_range_bins
            or message.sample_bit_depth not in (8, 16)
            or message.header.stamp.nanosec >= 1_000_000_000
            or message.host_stamp.nanosec >= 1_000_000_000
            or not message.header.frame_id or message.header.frame_id.startswith('/')
            or (self._output_frame and message.header.frame_id != self._output_frame)
        ):
            self._reject_spoke('Invalid FFT payload, timestamp, angle or frame')
            return
        # DDS does not order deliveries across configuration and spoke topics.
        # Never label data using geometry belonging to another version.
        if message.configuration_version != self._configuration_version:
            self._reject_spoke('FFT configuration version is not available')
            return
        if self._configuration_key is not None and (
            len(message.data) != self._range_bins
            or message.azimuth >= self._encoder_size
            or message.sample_bit_depth != self._sample_bit_depth
            or message.header.frame_id != self._configuration_frame
        ):
            self._reject_spoke('FFT spoke does not match its configuration')
            return

        # Validate before changing assembly state. ROS uint16 sequences own
        # their buffers; the view keeps the payload alive without a copy.
        values = np.asarray(message.data, dtype=np.uint16)
        if message.sample_bit_depth == 8 and np.any(values > 255):
            self._reject_spoke('8-bit FFT spoke contains values above 255')
            return
        values.setflags(write=False)

        angle %= 2 * math.pi
        if self._encoder_size is not None:
            encoder_angle = math.tau * message.azimuth / self._encoder_size
            if abs(math.remainder(angle - encoder_angle, math.tau)) > math.pi / self._encoder_size + 1e-12:
                self._reject_spoke('FFT spoke angle disagrees with encoder count')
                return
            angle = encoder_angle
        if self._spokes:
            previous = self._spokes[-1]
            gap_ns = ((message.header.stamp.sec - previous.stamp.sec) * 1_000_000_000
                      + message.header.stamp.nanosec - previous.stamp.nanosec)
            forward_jump = (gap_ns * self._rotation_rate >= 1_000_000_000 if self._rotation_rate
                            else gap_ns * 1e-9 > self._fallback_gap)
            if gap_ns < 0 or forward_jump:
                self._reset_sweep()
                if self._deskew:
                    self._deskew.clear()
            elif (
                len(message.data) != previous.values.size
                or message.sample_bit_depth != previous.sample_bit_depth
                or message.header.frame_id != previous.frame_id
            ):
                self._finish_sweep()
        if (
            self._last_angle is not None
            and angle < self._last_angle - self._wrap_threshold
        ):
            self._finish_sweep()

        if len(self._spokes) >= self._max_buffered_spokes:
            self.get_logger().warning(
                'Sweep buffer limit reached without an azimuth wrap; resetting',
                throttle_duration_sec=5.0,
            )
            self._reset_sweep()

        self._spokes.append(StoredSpoke(
            angle=angle,
            azimuth=int(message.azimuth),
            sweep_counter=int(message.sweep_counter),
            configuration_version=int(message.configuration_version),
            sample_bit_depth=int(message.sample_bit_depth),
            stamp=message.header.stamp,
            host_stamp=message.host_stamp,
            frame_id=message.header.frame_id,
            values=values,
        ))
        self._last_angle = angle

    def _reject_spoke(self, reason: str) -> None:
        self._invalid_spokes += 1
        self.get_logger().warning(reason, throttle_duration_sec=5.0)

    def _reset_sweep(self) -> None:
        if self._spokes:
            self._dropped_sweeps += 1
        self._spokes = []
        self._last_angle = None

    def _finish_sweep(self) -> None:
        spokes = self._spokes
        self._spokes = []
        self._last_angle = None
        if not spokes:
            return
        if len(spokes) < self._minimum_spokes:
            self._dropped_sweeps += 1
            return

        cells = {encoder_cell(s.azimuth, self._expected_spokes, self._encoder_size)
                 if self._encoder_size is not None else s.azimuth for s in spokes}
        missing_spokes = max(0, self._expected_spokes - len(cells))
        duplicate_spokes = len(spokes) - len(cells)
        if not self._publish_partial:
            scan_cells = {scan_cell(s.angle, spokes[0].angle, self._expected_spokes) for s in spokes}
            if (len(spokes) != self._expected_spokes or missing_spokes or duplicate_spokes
                    or None in scan_cells or len(scan_cells) != self._expected_spokes):
                self._dropped_sweeps += 1
                return

        # Stable sorting preserves duplicate encoder rows and their metadata.
        ordered = sorted(spokes, key=lambda spoke: spoke.azimuth)
        width = ordered[0].values.size

        pixels = np.empty((len(ordered), width), dtype='<u2')
        for row, spoke in enumerate(ordered):
            pixels[row, :] = spoke.values

        image = Image()
        image.header.stamp = spokes[0].stamp
        image.header.frame_id = spokes[0].frame_id
        image.height = pixels.shape[0]
        image.width = pixels.shape[1]
        image.encoding = 'mono16'
        image.is_bigendian = False
        image.step = image.width * 2
        image.data = array('B')
        image.data.frombytes(memoryview(pixels).cast('B'))

        sweep = FftSweep()
        sweep.header = image.header
        sweep.end_stamp = spokes[-1].stamp
        sweep.configuration_version = spokes[0].configuration_version
        sweep.sweep_sequence = self._sweep_sequence
        sweep.sample_bit_depth = spokes[0].sample_bit_depth
        sweep.range_resolution = self._range_resolution
        sweep.image = image
        sweep.encoder_values = [spoke.azimuth for spoke in ordered]
        sweep.sweep_counters = [spoke.sweep_counter for spoke in ordered]
        sweep.timestamps = [spoke.stamp for spoke in ordered]
        sweep.host_stamps = [spoke.host_stamp for spoke in ordered]

        self._sweep_publisher.publish(sweep)
        self._image_publisher.publish(image)
        # Extract once for both derived products. Scan selection precedes the
        # cloud's point budget so a strong distant return cannot hide a near one.
        peaks = ([self._extract_peak_bins(s.values) for s in ordered]
                 if self._publish_scan or self._publish_point_cloud else None)
        point_count = self._publish_cloud(image.header, ordered, peaks)
        if self._publish_scan:
            self._publish_laser_scan(image.header, spokes[0], ordered, peaks)

        self._sweep_sequence = (self._sweep_sequence + 1) % (1 << 32)
        if self._sweep_sequence == 1 or self._sweep_sequence % 40 == 0:
            self.get_logger().info(
                f'Published sweep {self._sweep_sequence - 1}: '
                f'{image.width}x{image.height} mono16, '
                f'{missing_spokes} missing, {duplicate_spokes} duplicate, '
                f'{point_count} selected points'
            )

    def _extract_peak_bins(self, values: np.ndarray) -> np.ndarray:
        """Local maxima, optional additive CA-CFAR, then range suppression.

        Training/guard counts are per side. Use complete windows only; raw FFT
        levels are not calibrated linear power, so no false-alarm probability
        or multiplicative power-domain threshold is implied.
        """
        if values.size < 3 or self._range_resolution <= 0.0:
            return np.empty(0, dtype=np.int64)

        first, stop = self._range_axis.window(values.size, self._min_range, self._max_range)
        first_bin, last_bin = max(1, first), min(values.size - 1, stop)
        if last_bin <= first_bin:
            return np.empty(0, dtype=np.int64)

        center = values[first_bin:last_bin]
        peak_mask = (
            (center >= self._threshold)
            & (center > values[first_bin - 1:last_bin - 1])
            & (center >= values[first_bin + 1:last_bin + 1])
        )
        bins = np.flatnonzero(peak_mask) + first_bin
        if self._peak_selection == 'cfar' and bins.size:
            guard, train = self._cfar_guard, self._cfar_training
            margin = guard + train
            bins = bins[(bins >= margin) & (bins < values.size - margin)]
            sums = np.concatenate(([0.0], np.cumsum(values, dtype=np.float64)))
            noise = (sums[bins - guard] - sums[bins - guard - train]
                     + sums[bins + guard + train + 1] - sums[bins + guard + 1]) / (2 * train)
            bins = bins[values[bins] > noise + self._cfar_offset]
            # Collapse nearby maxima into the strongest reflector. Sorting is
            # stable so equal levels deterministically prefer the nearer bin.
            if bins.size and self._peak_separation > 0:
                ranked = bins[np.argsort(-values[bins].astype(np.int32), kind='stable')]
                suppressed = np.zeros(values.size, dtype=bool)
                separation = int(math.ceil(min(
                    self._peak_separation / self._range_axis.spacing, values.size
                )))
                kept = []
                for index in ranked:
                    if not suppressed[index]:
                        kept.append(index)
                        suppressed[max(0, index - separation + 1):index + separation] = True
                bins = np.asarray(sorted(kept), dtype=np.int64)
        return bins

    def _extract_spoke_points(
        self, values: np.ndarray, angle: float, bins=None,
    ) -> np.ndarray | None:
        bins = self._extract_peak_bins(values) if bins is None else bins
        if bins.size == 0:
            return None

        if self._max_points > 0 and bins.size > self._max_points:
            intensities = values[bins]
            keep = np.argsort(-intensities.astype(np.int32), kind='stable')[:self._max_points]
            bins = np.sort(bins[keep])

        ranges = self._range_axis.metres(bins.astype(np.float64)).astype(np.float32)
        bearing = ros_bearing(angle, self._azimuth_offset)
        points = np.empty((bins.size, 4), dtype=np.float32)
        points[:, 0] = ranges * math.cos(bearing)
        points[:, 1] = ranges * math.sin(bearing)
        points[:, 2] = 0.0
        points[:, 3] = values[bins]
        return points

    def _publish_cloud(self, header, spokes: list[StoredSpoke], peaks=None) -> int:
        if not self._publish_point_cloud:
            return 0

        # Do projection only for sweeps that pass the publication policy.
        chunks = []
        for index, spoke in enumerate(spokes):
            points = self._extract_spoke_points(spoke.values, spoke.angle,
                                                None if peaks is None else peaks[index])
            if points is not None:
                chunks.append((spoke.stamp, points))
        if self._deskew:
            self._deskew.submit(header, chunks)
        else:
            self._publish_cloud_message(header, [points for _, points in chunks])
        return sum(len(points) for _, points in chunks)

    def _publish_cloud_message(self, header, spoke_points):
        points = (
            np.concatenate(spoke_points) if spoke_points
            else np.empty((0, 4), dtype='<f4')
        )
        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = int(points.shape[0])
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name='intensity',
                offset=12,
                datatype=PointField.FLOAT32,
                count=1,
            ),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = cloud.point_step * cloud.width
        cloud.data = array('B')
        cloud.data.frombytes(
            memoryview(points.astype('<f4', copy=False).reshape(-1)).cast('B')
        )
        cloud.is_dense = True
        self._point_cloud_publisher.publish(cloud)

    def _publish_laser_scan(self, header, first, spokes, peaks) -> None:
        """One detection per bearing; no angular interpolation across gaps.

        Anchor the angle grid at the first acquired spoke so the header is the
        acquisition time of ray zero, even for an initial partial revolution.
        """
        count = self._expected_spokes
        increment = 2 * math.pi / count
        scan = LaserScan()
        scan.header = header
        scan.angle_min = float(ros_bearing(first.angle, self._azimuth_offset) % (2 * math.pi))
        scan.angle_increment = -increment
        scan.angle_max = scan.angle_min + (count - 1) * scan.angle_increment
        scan.scan_time = 1.0 / self._rotation_rate if self._rotation_rate else 0.0
        scan.range_min = max(self._min_range, self._range_axis.offset)
        scan.range_max = min(self._max_range, self._range_axis.metres(first.values.size - 1))
        if scan.range_max < scan.range_min:
            return
        # A filtered radar ray supplies no evidence of clear space.
        ranges = np.full(count, np.nan, dtype=np.float32)
        intensities = np.zeros(count, dtype=np.float32)
        observed = np.zeros(count, dtype=bool)
        timing_valid = bool(self._rotation_rate)
        nominal_dt = scan.scan_time / count
        first_ns = first.stamp.sec * 1_000_000_000 + first.stamp.nanosec
        for spoke, bins in zip(spokes, peaks):
            index = scan_cell(spoke.angle, first.angle, count)
            if index is None:
                timing_valid = False
                continue
            stamp_ns = spoke.stamp.sec * 1_000_000_000 + spoke.stamp.nanosec
            timing_valid &= (not observed[index] and abs(
                (stamp_ns - first_ns) * 1e-9 - index * nominal_dt
            ) <= max(1e-6, nominal_dt * 0.1))
            observed[index] = True
            if not bins.size:
                continue
            selected = (bins[0] if self._scan_selection == 'nearest'
                        else bins[np.argmax(spoke.values[bins])])
            distance = self._range_axis.metres(int(selected))
            intensity = float(spoke.values[selected])
            if (not math.isfinite(float(ranges[index]))
                or (self._scan_selection == 'nearest' and distance < ranges[index])
                or (self._scan_selection == 'strongest' and intensity > intensities[index])):
                ranges[index], intensities[index] = distance, intensity
        # Irregular or duplicate timestamps describe a snapshot, not a made-up
        # constant-rate acquisition trajectory for downstream motion correction.
        scan.time_increment = nominal_dt if timing_valid else 0.0
        scan.ranges = array('f', ranges)
        scan.intensities = array('f', intensities)
        self._scan_publisher.publish(scan)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = FftSweepAssembler()
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
