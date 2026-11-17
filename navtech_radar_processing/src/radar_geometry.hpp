// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cmath>
#include <cstdint>
#include <utility>

#include "navtech_radar_msgs/msg/radar_configuration.hpp"

namespace radar {

// Encoder angles increase clockwise; yaw offsets are positive about ROS +z.
inline double ros_bearing(double encoder_angle, double yaw_offset) {
  return yaw_offset - encoder_angle;
}

// The grid is centred on encoder zero. Half-cell ties advance clockwise.
inline int encoder_cell(unsigned encoder, int count, unsigned size) {
  return ((uint64_t(2) * encoder * count + size) / (2 * size)) % count;
}

struct RangeAxis {
  double spacing;
  double offset;

  static RangeAxis
  from_config(const navtech_radar_msgs::msg::RadarConfiguration &c) {
    // Older canonical publishers left both optional coefficients at zero.
    const double gain =
        c.range_gain == 0 && c.range_offset == 0 ? 1.0 : c.range_gain;
    return {c.range_resolution * gain, c.range_offset};
  }

  bool valid(int bins) const {
    return spacing > 0 && std::isfinite(spacing) && std::isfinite(offset) &&
           bins > 0 && std::isfinite(metres(bins - 1));
  }

  double metres(int bin) const { return bin * spacing + offset; }

  // Compare in metres to include exact endpoints even when division by the
  // calibrated spacing would round a boundary to the neighbouring bin.
  std::pair<int, int> window(int bins, double minimum, double maximum) const {
    auto bound = [&](double target, bool upper) {
      int lo = 0, hi = bins;
      while (lo < hi) {
        const int mid = lo + (hi - lo) / 2;
        const double r = metres(mid);
        if (upper ? r <= target : r < target)
          lo = mid + 1;
        else
          hi = mid;
      }
      return lo;
    };
    return {bound(minimum, false), bound(maximum, true)};
  }
};
} // namespace radar
