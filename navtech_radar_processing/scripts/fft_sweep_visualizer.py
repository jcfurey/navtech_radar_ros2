#!/usr/bin/env python3
"""Produce display images from lossless sweeps without changing the raw data."""

from array import array
import math

import cv2
from navtech_radar_msgs.msg import FftSweep, RadarConfiguration
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Image


class SweepRenderer:
    """Range pooling, stable contrast and encoder-based plan-view projection."""

    def __init__(
        self, min_range=2.0, max_range=150.0, image_width=1000,
        cartesian_size=800, lower_percentile=50.0, upper_percentile=99.7,
        contrast_alpha=0.2, gamma=1.0, annotate=True,
    ):
        finite = (min_range, max_range, lower_percentile, upper_percentile,
                  contrast_alpha, gamma)
        if not all(math.isfinite(value) for value in finite):
            raise ValueError('Display parameters must be finite')
        if not 0 <= min_range < max_range:
            raise ValueError('Require 0 <= min_range < max_range')
        if not 64 <= image_width <= 4096 or not 64 <= cartesian_size <= 2048:
            raise ValueError('Require image_width in [64,4096], cartesian_size in [64,2048]')
        if not 0 <= lower_percentile < upper_percentile <= 100:
            raise ValueError('Require 0 <= lower_percentile < upper_percentile <= 100')
        if not 0 < contrast_alpha <= 1 or not 0 < gamma <= 5:
            raise ValueError('Require contrast_alpha in (0,1] and gamma in (0,5]')
        self.min_range = min_range
        self.max_range = max_range
        self.image_width = image_width
        self.cartesian_size = cartesian_size
        self.lower_percentile = lower_percentile
        self.upper_percentile = upper_percentile
        self.contrast_alpha = contrast_alpha
        self.gamma = gamma
        self.annotate = annotate
        self._bounds = None
        self._epoch = None
        self._stamp = None
        self._map_key = None
        self._map = None
        self._colors = cv2.applyColorMap(
            np.arange(256, dtype=np.uint8), cv2.COLORMAP_INFERNO
        ).reshape(256, 3)
        self._colors[0] = 0

    @staticmethod
    def validate_configuration(config):
        if (
            config.configuration_version == 0
            or not 0 < config.azimuth_samples <= min(2048, config.encoder_size)
            or not 0 < config.range_bins <= 16384
            or not math.isfinite(config.range_resolution) or config.range_resolution <= 0
            or not config.header.frame_id
        ):
            raise ValueError('Invalid display configuration or image dimensions exceed limits')

    def render(self, sweep, config):
        """Return annotated BGR8 polar and Cartesian images for one sweep."""
        self.validate_configuration(config)
        image = sweep.image
        if (
            sweep.configuration_version != config.configuration_version
            or sweep.header.frame_id != config.header.frame_id
            or sweep.range_resolution != config.range_resolution
            or image.encoding != 'mono16'
            or not 0 < image.height <= 2048 or image.width != config.range_bins
            or image.step < image.width * 2 or image.step % 2
            or len(image.data) != image.height * image.step
            or len(sweep.encoder_values) != image.height
            or any(value >= config.encoder_size for value in sweep.encoder_values)
        ):
            raise ValueError('Sweep image or encoder metadata does not match configuration')

        resolution = sweep.range_resolution
        # Clamp before conversion to int, including extreme but finite resolutions.
        first = int(math.ceil(min(self.min_range / resolution, image.width)))
        last = min(image.width, int(math.floor(min(
            self.max_range / resolution, image.width
        ))) + 1)
        if first >= last:
            raise ValueError('Display range does not overlap the FFT bins')
        pixels = np.ndarray(
            (image.height, image.width),
            dtype='>u2' if image.is_bigendian else '<u2',
            buffer=image.data, strides=(image.step, 2),
        )
        stride = max(1, math.ceil((last - first) / self.image_width))
        # Max pooling keeps narrow peaks that ordinary image resizing can hide.
        pooled = np.maximum.reduceat(
            pixels[:, first:last], np.arange(0, last - first, stride), axis=1
        )
        row_indices = np.rint(
            np.asarray(sweep.encoder_values, dtype=np.float64)
            * config.azimuth_samples / config.encoder_size
        ).astype(np.int32) % config.azimuth_samples
        grid = np.zeros((config.azimuth_samples, pooled.shape[1]), dtype=np.uint16)
        np.maximum.at(grid, row_indices, pooled)
        present = np.zeros(config.azimuth_samples, dtype=bool)
        present[row_indices] = True

        epoch = (config.configuration_version, config.header.frame_id,
                 config.encoder_size, config.azimuth_samples, image.width, resolution)
        stamp = (sweep.header.stamp.sec, sweep.header.stamp.nanosec)
        if epoch != self._epoch or (self._stamp is not None and stamp < self._stamp):
            self._bounds = None
        self._epoch, self._stamp = epoch, stamp
        bounds = np.percentile(grid[present], [self.lower_percentile, self.upper_percentile])
        self._bounds = bounds if self._bounds is None else (
            (1 - self.contrast_alpha) * self._bounds + self.contrast_alpha * bounds
        )
        low, high = self._bounds
        scaled = np.clip((grid.astype(np.float32) - low) / max(high - low, 1), 0, 1)
        scaled = np.rint(np.power(scaled, self.gamma) * 255).astype(np.uint8)
        scaled[~present] = 0
        polar = self._colors[scaled]

        extent = min(self.max_range, (image.width - 1) * resolution)
        key = (grid.shape, first, last, stride, resolution, extent)
        if key != self._map_key:
            self._map_key = key
            center = (self.cartesian_size - 1) / 2
            axis = (center - np.arange(self.cartesian_size)) * extent / center
            x, y = np.meshgrid(axis, axis, indexing='ij')
            ranges = np.hypot(x, y)
            angles = np.arctan2(y, x) % (2 * math.pi)
            rows = np.rint(angles * grid.shape[0] / (2 * math.pi)).astype(np.int32)
            rows %= grid.shape[0]
            columns = np.floor((ranges / resolution - first) / stride)
            valid = (ranges >= self.min_range) & (ranges <= extent) & (columns >= 0)
            # Clipping before the cast also handles extremely small resolutions.
            columns = np.clip(columns, 0, grid.shape[1] - 1).astype(np.int32)
            self._map = (rows, columns, valid)
        rows, columns, valid = self._map
        cartesian = self._colors[scaled[rows, columns]]
        cartesian[~valid] = 0
        if self.annotate:
            polar = self._annotate_polar(polar, first * resolution, (last - 1) * resolution)
            self._annotate_cartesian(cartesian, extent)
        return polar, cartesian

    @staticmethod
    def _text(image, text, position, scale=0.45):
        cv2.putText(image, text, position, cv2.FONT_HERSHEY_SIMPLEX,
                    scale, (220, 220, 220), 1, cv2.LINE_AA)

    def _annotate_polar(self, image, minimum, maximum):
        height, width = image.shape[:2]
        result = cv2.copyMakeBorder(image, 30, 56, 48, 18, cv2.BORDER_CONSTANT)
        self._text(result, 'Azimuth (deg) / range (m)', (48, 20))
        for angle in (0, 90, 180, 270):
            row = 30 + round(angle * height / 360)
            self._text(result, str(angle), (5, row + 5))
        for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
            col = 48 + round(fraction * (width - 1))
            self._text(result, f'{minimum + fraction * (maximum - minimum):.0f}',
                       (min(col, width + 36), height + 48))
        low, high = self._bounds
        self._text(result, f'FFT level: {low:.0f} (dark) - {high:.0f} (bright)',
                   (48, height + 75))
        return result

    def _annotate_cartesian(self, image, extent):
        center = (self.cartesian_size - 1) // 2
        for fraction in (1 / 3, 2 / 3, 1):
            radius = max(1, round(center * fraction))
            cv2.circle(image, (center, center), radius, (65, 65, 65), 1, cv2.LINE_AA)
            self._text(image, f'{extent * fraction:.0f} m',
                       (center + 5, max(35, center - radius + 15)))
        cv2.arrowedLine(image, (center, center), (center, center - 30), (180, 180, 180))
        cv2.arrowedLine(image, (center, center), (center - 30, center), (180, 180, 180))
        self._text(image, 'x', (center + 5, center - 30))
        self._text(image, 'y', (center - 40, center - 5))
        self._text(image, 'Radar plan view | x forward, y left', (12, 20))


class FftSweepVisualizer(Node):
    """Publish optional enhanced images; the raw sweep remains the source of truth."""

    def __init__(self, **kwargs):
        super().__init__('navtech_fft_sweep_visualizer', **kwargs)
        defaults = {
            'configuration_topic': 'navtech_radar/configuration',
            'sweep_topic': 'navtech_radar/fft/sweep',
            'polar_topic': 'navtech_radar/fft/display',
            'cartesian_topic': 'navtech_radar/fft/cartesian',
            'min_range': 2.0, 'max_range': 150.0,
            'image_width': 1000, 'cartesian_size': 800,
            'lower_percentile': 50.0, 'upper_percentile': 99.7,
            'contrast_alpha': 0.2, 'gamma': 1.0, 'annotate': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self._renderer = SweepRenderer(**{
            name: self.get_parameter(name).value for name in defaults
            if not name.endswith('_topic')
        })
        self._configuration = None
        self._polar_publisher = self.create_publisher(
            Image, self.get_parameter('polar_topic').value, 1
        )
        self._cartesian_publisher = self.create_publisher(
            Image, self.get_parameter('cartesian_topic').value, 1
        )
        self._configuration_subscription = self.create_subscription(
            RadarConfiguration, self.get_parameter('configuration_topic').value,
            self._configuration_callback,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self._sweep_subscription = self.create_subscription(
            FftSweep, self.get_parameter('sweep_topic').value, self._sweep_callback, 1
        )

    def _configuration_callback(self, message):
        try:
            self._renderer.validate_configuration(message)
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.0)
            return
        self._configuration = message

    def _sweep_callback(self, message):
        if self._configuration is None or not (
            self._polar_publisher.get_subscription_count()
            or self._cartesian_publisher.get_subscription_count()
        ):
            return
        try:
            polar, cartesian = self._renderer.render(message, self._configuration)
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.0)
            return
        for publisher, pixels in (
            (self._polar_publisher, polar), (self._cartesian_publisher, cartesian)
        ):
            if publisher.get_subscription_count():
                output = Image()
                output.header = message.header
                output.height, output.width = pixels.shape[:2]
                output.encoding = 'bgr8'
                output.step = output.width * 3
                output.data = array('B')
                output.data.frombytes(memoryview(np.ascontiguousarray(pixels)).cast('B'))
                publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FftSweepVisualizer()
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
