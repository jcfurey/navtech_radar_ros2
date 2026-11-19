"""Conversion from native radar coordinates to metric ROS display geometry."""
from bisect import bisect_left, bisect_right
from dataclasses import dataclass
import math


def ros_bearing(encoder_angle, yaw_offset):
    """Clockwise native encoder angle to a right-handed ROS bearing."""
    return yaw_offset - encoder_angle


def encoder_cell(encoder, count, size):
    """Index a scalar or int64 array of counts, advancing clockwise on ties."""
    return ((2 * encoder * count + size) // (2 * size)) % count


def scan_cell(angle, start, count):
    position = ((angle - start) % math.tau) / (math.tau / count)
    nearest = round(position)
    return nearest % count if abs(position - nearest) <= 0.25 else None


@dataclass(frozen=True)
class RangeAxis:
    spacing: float
    offset: float

    @classmethod
    def from_config(cls, config):
        gain = config.range_gain
        if gain == 0 and config.range_offset == 0:
            gain = 1.0  # Older canonical publishers omitted both coefficients.
        return cls(config.range_resolution * gain, config.range_offset)

    def valid(self, bins):
        return (self.spacing > 0 and math.isfinite(self.spacing) and math.isfinite(self.offset)
                and bins > 0 and math.isfinite(self.metres(bins - 1)))

    def metres(self, bin_index):
        return bin_index * self.spacing + self.offset

    def window(self, bins, minimum, maximum):
        indices = range(bins)
        return (bisect_left(indices, minimum, key=self.metres),
                bisect_right(indices, maximum, key=self.metres))
