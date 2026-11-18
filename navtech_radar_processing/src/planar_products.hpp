// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "navtech_radar_msgs/msg/fft_spoke.hpp"
#include "radar_geometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace radar {
using Spoke = navtech_radar_msgs::msg::FftSpoke;
using Scan = sensor_msgs::msg::LaserScan;
using Cloud = sensor_msgs::msg::PointCloud2;
constexpr double tau = 6.283185307179586476925286766559;

inline double positive_angle(double angle) {
  const auto remainder = std::fmod(angle, tau);
  return remainder < 0 ? remainder + tau : remainder;
}

inline int64_t nanoseconds(const builtin_interfaces::msg::Time &t) {
  return static_cast<int64_t>(t.sec) * 1000000000 + t.nanosec;
}

// Use the same relative grid and tolerance in strict assembly and LaserScan.
inline std::optional<size_t> scan_cell(double angle, double start, int count) {
  const double position = positive_angle(angle - start) / (tau / count);
  const auto cell = static_cast<size_t>(std::llround(position));
  if (std::abs(position - cell) > 0.25)
    return {};
  return cell % count;
}

struct DetectionSettings {
  double min_range, max_range, cfar_offset, separation;
  int threshold, training, guard, cloud_limit;
  bool cfar, nearest;
};

// Reuse scratch space across spokes.
class PeakDetector {
public:
  const std::vector<int> &select(const std::vector<uint16_t> &samples,
                                 const RangeAxis &axis,
                                 const DetectionSettings &p) {
    const int width = static_cast<int>(samples.size());
    const int margin = p.cfar ? p.training + p.guard : 1;
    const auto [start, stop] = axis.window(width, p.min_range, p.max_range);
    const int first = std::max(margin, start);
    const int end = std::min(width - margin, stop);
    bins_.clear();
    if (first >= end)
      return bins_;
    const auto *data = samples.data();
    constexpr int block_size = 32;
    for (int begin = first; begin < end; begin += block_size) {
      const int block_end = std::min(begin + block_size, end);
      if (*std::max_element(data + begin, data + block_end) < p.threshold)
        continue;
      for (int i = begin; i < block_end; ++i) {
        const auto value = data[i];
        if (value >= p.threshold && value > data[i - 1] && value >= data[i + 1])
          bins_.push_back(i);
      }
    }
    if (p.cfar && !bins_.empty()) {
      // Sum the training windows directly for sparse peaks. A prefix sum
      // saves work once their combined length exceeds the search interval.
      const bool dense = bins_.size() * (2 * p.training) >
                         static_cast<size_t>(end - first + 2 * margin);
      if (dense) {
        sums_.resize(width + 1);
        sums_[first - margin] = 0;
        for (int i = first - margin; i < end + margin; ++i)
          sums_[i + 1] = sums_[i] + samples[i];
      }
      auto rejected = [&](int i) {
        const auto sum =
            dense ? (sums_[i - p.guard] - sums_[i - margin]) +
                        (sums_[i + margin + 1] - sums_[i + p.guard + 1])
                  : std::accumulate(data + i - margin, data + i - p.guard, 0u) +
                        std::accumulate(data + i + p.guard + 1,
                                        data + i + margin + 1, 0u);
        return data[i] <= double(sum) / (2 * p.training) + p.cfar_offset;
      };
      bins_.erase(std::remove_if(bins_.begin(), bins_.end(), rejected),
                  bins_.end());
    }
    const int separation = static_cast<int>(
        std::ceil(std::min(double(width), p.separation / axis.spacing)));
    if (p.cfar && separation > 1 && bins_.size() > 1) {
      // Sorting integer keys keeps equal-strength peaks in ascending bin order.
      // Both the FFT level and a bin index fit in 16 bits on the wire.
      priority_.clear();
      for (int i : bins_)
        priority_.push_back((uint32_t(UINT16_MAX - samples[i]) << 16) |
                            uint32_t(i));
      std::sort(priority_.begin(), priority_.end());
      enum : uint8_t { available, suppressed, retained };
      blocked_.assign(width, available);
      for (uint32_t key : priority_) {
        const int i = key & UINT16_MAX;
        if (blocked_[i])
          continue;
        std::fill(blocked_.begin() + std::max(0, i - separation + 1),
                  blocked_.begin() + std::min(width, i + separation),
                  suppressed);
        blocked_[i] = retained;
      }
      bins_.erase(
          std::remove_if(bins_.begin(), bins_.end(),
                         [&](int i) { return blocked_[i] != retained; }),
          bins_.end());
    }
    return bins_;
  }

private:
  std::vector<int> bins_;
  std::vector<uint64_t> sums_;
  std::vector<uint8_t> blocked_;
  std::vector<uint32_t> priority_;
};

inline bool big_endian() {
  const uint16_t word = 1;
  return *reinterpret_cast<const uint8_t *>(&word) == 0;
}

inline Cloud empty_cloud(const std_msgs::msg::Header &header) {
  Cloud cloud;
  cloud.header = header;
  cloud.height = 1;
  cloud.point_step = 4 * sizeof(float);
  cloud.is_dense = true;
  cloud.is_bigendian = big_endian();
  for (auto name : {"x", "y", "z", "intensity"}) {
    sensor_msgs::msg::PointField f;
    f.name = name;
    f.offset = cloud.fields.size() * sizeof(float);
    f.count = 1;
    f.datatype = f.FLOAT32;
    cloud.fields.push_back(std::move(f));
  }
  return cloud;
}

inline void append_points(Cloud &cloud, const Spoke &s,
                          const std::vector<int> &bins, const RangeAxis &axis,
                          double offset, int limit, std::vector<int> &scratch) {
  const auto *selected = &bins;
  if (limit > 0 && bins.size() > static_cast<size_t>(limit)) {
    scratch = bins;
    const auto boundary = scratch.begin() + limit;
    std::nth_element(
        scratch.begin(), boundary, scratch.end(), [&](int a, int b) {
          return s.data[a] == s.data[b] ? a < b : s.data[a] > s.data[b];
        });
    scratch.resize(limit);
    std::sort(scratch.begin(), scratch.end());
    selected = &scratch;
  }
  const double bearing = ros_bearing(s.azimuth_angle, offset);
  const double x = std::cos(bearing), y = std::sin(bearing);
  auto pos = cloud.data.size();
  cloud.data.resize(pos + selected->size() * cloud.point_step);
  for (const auto i : *selected) {
    const double r = axis.metres(i);
    const std::array<float, 4> point = {static_cast<float>(r * x),
                                        static_cast<float>(r * y), 0,
                                        static_cast<float>(s.data[i])};
    std::memcpy(cloud.data.data() + pos, point.data(), cloud.point_step);
    pos += cloud.point_step;
  }
  cloud.width += selected->size();
  cloud.row_step = cloud.data.size();
}

class ScanBuilder {
public:
  ScanBuilder(const Spoke &first, int count, const RangeAxis &axis, int rate,
              double offset, const DetectionSettings &p)
      : start_(first.azimuth_angle),
        first_stamp_(nanoseconds(first.header.stamp)), increment_(tau / count),
        dt_(rate ? 1.0 / (double(rate) * count) : 0.0), regular_(rate != 0),
        observed_(count, false) {
    scan.header = first.header;
    const double first_bearing = positive_angle(ros_bearing(start_, offset));
    scan.angle_min = first_bearing;
    scan.angle_max = first_bearing - (count - 1) * increment_;
    scan.angle_increment = -increment_;
    scan.scan_time = rate ? 1.0 / rate : 0.0;
    scan.range_min = std::max(p.min_range, axis.offset);
    scan.range_max = std::min(p.max_range, axis.metres(first.data.size() - 1));
    scan.ranges.assign(count, std::numeric_limits<float>::quiet_NaN());
    scan.intensities.assign(count, 0);
  }
  void add(const Spoke &s, const std::vector<int> &bins, const RangeAxis &axis,
           bool nearest) {
    const auto cell = scan_cell(s.azimuth_angle, start_, scan.ranges.size());
    if (!cell) {
      regular_ = false;
      return;
    }
    const auto i = *cell;
    auto &range = scan.ranges[i];
    auto &level = scan.intensities[i];
    const auto elapsed = (nanoseconds(s.header.stamp) - first_stamp_) * 1e-9;
    regular_ = regular_ && !observed_[i] &&
               std::abs(elapsed - i * dt_) <= std::max(1e-6, dt_ * 0.1);
    observed_[i] = true;
    if (bins.empty())
      return;
    const int best = nearest ? bins.front()
                             : *std::max_element(bins.begin(), bins.end(),
                                                 [&](int a, int b) {
                                                   return s.data[a] < s.data[b];
                                                 });
    const float distance = axis.metres(best);
    if (!std::isfinite(range) ||
        (nearest ? distance < range : s.data[best] > level)) {
      range = distance;
      level = s.data[best];
    }
  }
  const Scan &finish() {
    scan.time_increment = regular_ ? dt_ : 0;
    return scan;
  }

private:
  Scan scan;
  double start_;
  int64_t first_stamp_;
  double increment_, dt_;
  bool regular_;
  std::vector<bool> observed_;
};
} // namespace radar
