"""Transform each point-bearing spoke to the radar pose at the cloud timestamp."""
import time

from rclpy.clock import Clock, ClockType
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_sensor_msgs import transform_points


class CloudDeskew:
    def __init__(self, node, fixed_frame, publish):
        self.node = node
        self.fixed_frame = fixed_frame
        self.publish = publish
        self.buffer = Buffer(node=node)
        self.listener = TransformListener(self.buffer, node, spin_thread=False)
        self.pending = None
        self.timer = node.create_timer(0.05, self.retry, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def clear(self):
        self.pending = None

    def submit(self, header, chunks):
        self.retry(superseded=True)
        if (header.frame_id == self.fixed_frame or (header.stamp.sec, header.stamp.nanosec) <= (0, 0)
                or any((stamp.sec, stamp.nanosec) <= (0, 0) for stamp, _ in chunks)):
            self.node.get_logger().warning(
                'Skipping deskewed cloud: need positive acquisition times and a distinct fixed frame',
                throttle_duration_sec=5.0)
            return
        self.pending = (header, chunks, time.monotonic() + 1.0)
        self.retry()

    def retry(self, *, superseded=False):
        if self.pending is None:
            return
        header, chunks, deadline = self.pending
        frame = header.frame_id
        reference = Time.from_msg(header.stamp)
        try:
            # The last spoke is usually the one still waiting for TF.
            transforms = [self.buffer.lookup_transform_full(
                frame, reference, frame, Time.from_msg(stamp), self.fixed_frame,
            ).transform for stamp, _ in reversed(chunks)]
        except TransformException as error:
            if superseded or time.monotonic() >= deadline:
                self.node.get_logger().warning(
                    f'Dropping cloud without deskew TF: {error}', throttle_duration_sec=5.0)
                self.clear()
            return
        for (_, points), transform in zip(chunks, reversed(transforms)):
            points[:, :3] = transform_points(points[:, :3], transform)
        self.publish(header, [points for _, points in chunks])
        self.clear()
