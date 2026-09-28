#!/usr/bin/env python3
"""Assemble lossless Navtech FFT spokes into metadata-rich revolutions."""

from dataclasses import dataclass
import math

import numpy as np
import rclpy
from navtech_radar_msgs.msg import FftSpoke, FftSweep, RadarConfiguration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, PointField


@dataclass
class StoredSpoke:
    arrival_index: int
    angle: float
    azimuth: int
    sweep_counter: int
    configuration_version: int
    sample_bit_depth: int
    stamp: object
    host_stamp: object
    values: np.ndarray
    points: np.ndarray | None


class FftSweepAssembler(Node):
    """Create sweep and visualization products from canonical FFT spokes."""

    def __init__(self) -> None:
        super().__init__('navtech_fft_sweep_assembler')

        self.declare_parameter(
            'configuration_topic', '/navtech_radar/configuration'
        )
        self.declare_parameter('spoke_topic', '/navtech_radar/fft/spoke')
        self.declare_parameter('sweep_topic', '/navtech_radar/fft/sweep')
        self.declare_parameter('image_topic', '/navtech_radar/fft/image')
        self.declare_parameter('point_cloud_topic', '/navtech_radar/point_cloud')
        self.declare_parameter('output_frame', 'navtech')
        self.declare_parameter('range_resolution', 0.0438)
        self.declare_parameter('expected_spokes', 400)
        self.declare_parameter('publish_partial_sweeps', True)
        self.declare_parameter('minimum_spokes', 2)
        self.declare_parameter('publish_point_cloud', False)
        self.declare_parameter('min_range', 2.0)
        self.declare_parameter('max_range', 200.0)
        self.declare_parameter('intensity_threshold', 22000)
        self.declare_parameter('max_points_per_spoke', 48)
        self.declare_parameter('azimuth_offset_degrees', 0.0)
        self.declare_parameter('wrap_threshold_degrees', 180.0)
        self.declare_parameter('input_queue_depth', 512)

        self._output_frame = str(self.get_parameter('output_frame').value)
        self._range_resolution = float(
            self.get_parameter('range_resolution').value
        )
        self._expected_spokes = int(self.get_parameter('expected_spokes').value)
        self._publish_partial = bool(
            self.get_parameter('publish_partial_sweeps').value
        )
        self._minimum_spokes = int(self.get_parameter('minimum_spokes').value)
        self._publish_point_cloud = bool(
            self.get_parameter('publish_point_cloud').value
        )
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

        self._configuration_version = 0
        self._last_angle = None
        self._spokes: list[StoredSpoke] = []
        self._arrival_index = 0
        self._sweep_sequence = 0
        self._dropped_sweeps = 0

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

    def _configuration_callback(self, message: RadarConfiguration) -> None:
        if (
            self._spokes
            and self._configuration_version
            and message.configuration_version != self._configuration_version
        ):
            self._finish_sweep()

        self._configuration_version = int(message.configuration_version)
        if math.isfinite(message.range_resolution) and message.range_resolution > 0:
            self._range_resolution = float(message.range_resolution)
        if message.azimuth_samples > 0:
            self._expected_spokes = int(message.azimuth_samples)

        self.get_logger().info(
            f'Using configuration version {self._configuration_version}: '
            f'{self._expected_spokes} spokes, {message.range_bins} bins, '
            f'{self._range_resolution:.6f} m/bin'
        )

    def _spoke_callback(self, message: FftSpoke) -> None:
        angle = float(message.azimuth_angle)
        if not math.isfinite(angle) or not message.data:
            return

        if (
            self._last_angle is not None
            and angle < self._last_angle - self._wrap_threshold
        ):
            self._finish_sweep()

        values = np.asarray(message.data, dtype=np.uint16).copy()
        points = None
        if self._publish_point_cloud:
            points = self._extract_spoke_points(values, angle)

        self._spokes.append(StoredSpoke(
            arrival_index=self._arrival_index,
            angle=angle,
            azimuth=int(message.azimuth),
            sweep_counter=int(message.sweep_counter),
            configuration_version=int(message.configuration_version),
            sample_bit_depth=int(message.sample_bit_depth),
            stamp=message.header.stamp,
            host_stamp=message.host_stamp,
            values=values,
            points=points,
        ))
        self._arrival_index += 1
        self._last_angle = angle

    def _finish_sweep(self) -> None:
        spokes = self._spokes
        self._spokes = []
        if len(spokes) < self._minimum_spokes:
            self._dropped_sweeps += 1
            return

        unique_azimuths = len({spoke.azimuth for spoke in spokes})
        missing_spokes = max(0, self._expected_spokes - unique_azimuths)
        duplicate_spokes = len(spokes) - unique_azimuths
        complete = missing_spokes == 0 and duplicate_spokes == 0
        if not complete and not self._publish_partial:
            self._dropped_sweeps += 1
            return

        ordered = sorted(
            spokes,
            key=lambda spoke: (spoke.angle, spoke.arrival_index),
        )
        width = min(spoke.values.size for spoke in ordered)
        if width == 0:
            self._dropped_sweeps += 1
            return

        pixels = np.empty((len(ordered), width), dtype='<u2')
        for row, spoke in enumerate(ordered):
            pixels[row, :] = spoke.values[:width]

        image = Image()
        image.header.stamp = spokes[0].stamp
        image.header.frame_id = self._output_frame
        image.height = pixels.shape[0]
        image.width = pixels.shape[1]
        image.encoding = 'mono16'
        image.is_bigendian = False
        image.step = image.width * 2
        image.data = pixels.tobytes()

        sweep = FftSweep()
        sweep.header = image.header
        sweep.end_stamp = spokes[-1].stamp
        sweep.configuration_version = max(
            spoke.configuration_version for spoke in ordered
        )
        sweep.sweep_sequence = self._sweep_sequence
        sweep.sample_bit_depth = max(
            spoke.sample_bit_depth for spoke in ordered
        )
        sweep.range_resolution = self._range_resolution
        sweep.image = image
        sweep.encoder_values = [spoke.azimuth for spoke in ordered]
        sweep.sweep_counters = [spoke.sweep_counter for spoke in ordered]
        sweep.timestamps = [spoke.stamp for spoke in ordered]
        sweep.host_stamps = [spoke.host_stamp for spoke in ordered]

        self._sweep_publisher.publish(sweep)
        self._image_publisher.publish(image)
        point_count = self._publish_cloud(spokes[-1].stamp, ordered)

        self._sweep_sequence += 1
        if self._sweep_sequence == 1 or self._sweep_sequence % 40 == 0:
            self.get_logger().info(
                f'Published sweep {self._sweep_sequence - 1}: '
                f'{image.width}x{image.height} mono16, '
                f'{missing_spokes} missing, {duplicate_spokes} duplicate, '
                f'{point_count} selected points'
            )

    def _extract_spoke_points(
        self, values: np.ndarray, angle: float
    ) -> np.ndarray | None:
        if values.size < 3 or self._range_resolution <= 0.0:
            return None

        first_bin = max(
            1, int(math.ceil(self._min_range / self._range_resolution))
        )
        last_bin = min(
            values.size - 1,
            int(math.floor(self._max_range / self._range_resolution)) + 1,
        )
        if last_bin <= first_bin:
            return None

        center = values[first_bin:last_bin]
        peak_mask = (
            (center >= self._threshold)
            & (center > values[first_bin - 1:last_bin - 1])
            & (center >= values[first_bin + 1:last_bin + 1])
        )
        bins = np.flatnonzero(peak_mask) + first_bin
        if bins.size == 0:
            return None

        if self._max_points > 0 and bins.size > self._max_points:
            intensities = values[bins]
            keep = np.argpartition(intensities, -self._max_points)[
                -self._max_points:
            ]
            bins = bins[keep]

        ranges = bins.astype(np.float32) * self._range_resolution
        bearing = angle + self._azimuth_offset
        points = np.empty((bins.size, 4), dtype=np.float32)
        points[:, 0] = ranges * math.cos(bearing)
        points[:, 1] = ranges * math.sin(bearing)
        points[:, 2] = 0.0
        points[:, 3] = values[bins]
        return points

    def _publish_cloud(self, stamp, spokes: list[StoredSpoke]) -> int:
        if not self._publish_point_cloud:
            return 0

        spoke_points = [
            spoke.points for spoke in spokes if spoke.points is not None
        ]
        if not spoke_points:
            return 0

        points = np.concatenate(spoke_points)
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = self._output_frame
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
        cloud.data = points.astype('<f4', copy=False).tobytes()
        cloud.is_dense = True
        self._point_cloud_publisher.publish(cloud)
        return cloud.width


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FftSweepAssembler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
