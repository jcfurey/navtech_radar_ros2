// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include "navtech_radar_msgs/msg/fft_spoke.hpp"
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

struct DetectionSettings {
  double min_range, max_range, cfar_offset, separation;
  int threshold, training, guard, cloud_limit;
  bool cfar, nearest;
};

// Reuse scratch space across spokes. A prefix sum gives the two training
// windows in constant time, with guard cells and the peak excluded.
class PeakDetector {
public:
  const std::vector<int> &select(const std::vector<uint16_t> &samples,
                                 double resolution,
                                 const DetectionSettings &p) {
    const int width = static_cast<int>(samples.size());
    const int margin = p.cfar ? p.training + p.guard : 1;
    const int first =
        std::max(margin, static_cast<int>(std::ceil(std::min(
                             double(width), p.min_range / resolution))));
    const int end = std::min(width - margin,
                             1 + static_cast<int>(std::min(
                                     double(width), p.max_range / resolution)));
    bins_.clear();
    if (first >= end)
      return bins_;
    if (p.cfar) {
      sums_.resize(width + 1);
      sums_[first - margin] = 0;
      for (int i = first - margin; i < end + margin; ++i)
        sums_[i + 1] = sums_[i] + samples[i];
    }
    for (int i = first; i < end; ++i) {
      const auto value = samples[i];
      if (value < p.threshold || value <= samples[i - 1] ||
          value < samples[i + 1])
        continue;
      if (p.cfar) {
        const auto sum = (sums_[i - p.guard] - sums_[i - margin]) +
                         (sums_[i + margin + 1] - sums_[i + p.guard + 1]);
        if (value <= double(sum) / (2 * p.training) + p.cfar_offset)
          continue;
      }
      bins_.push_back(i);
    }
    const int separation = static_cast<int>(
        std::ceil(std::min(double(width), p.separation / resolution)));
    if (p.cfar && separation > 1 && bins_.size() > 1) {
      std::sort(bins_.begin(), bins_.end(), [&](int a, int b) {
        return samples[a] == samples[b] ? a < b : samples[a] > samples[b];
      });
      blocked_.assign(width, false);
      auto kept = bins_.begin();
      for (int i : bins_) {
        if (blocked_[i])
          continue;
        *kept++ = i;
        std::fill(blocked_.begin() + std::max(0, i - separation + 1),
                  blocked_.begin() + std::min(width, i + separation), true);
      }
      bins_.erase(kept, bins_.end());
      std::sort(bins_.begin(), bins_.end());
    }
    return bins_;
  }

private:
  std::vector<int> bins_;
  std::vector<uint64_t> sums_;
  std::vector<uint8_t> blocked_;
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
                          const std::vector<int> &bins, double resolution,
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
  const float x = std::cos(s.azimuth_angle + offset),
              y = std::sin(s.azimuth_angle + offset);
  auto pos = cloud.data.size();
  cloud.data.resize(pos + selected->size() * cloud.point_step);
  for (const auto i : *selected) {
    const float r = static_cast<float>(i) * static_cast<float>(resolution);
    const std::array<float, 4> point = {r * x, r * y, 0,
                                        static_cast<float>(s.data[i])};
    std::memcpy(cloud.data.data() + pos, point.data(), cloud.point_step);
    pos += cloud.point_step;
  }
  cloud.width += selected->size();
  cloud.row_step = cloud.data.size();
}

class ScanBuilder {
public:
  ScanBuilder(const Spoke &first, int count, double resolution, int rate,
              double offset, const DetectionSettings &p)
      : start_(first.azimuth_angle),
        first_stamp_(nanoseconds(first.header.stamp)), increment_(tau / count),
        dt_(rate ? 1.0 / (double(rate) * count) : 0.0), regular_(rate != 0) {
    scan.header = first.header;
    scan.angle_min = positive_angle(start_ + offset);
    scan.angle_max = positive_angle(start_ + offset) + (count - 1) * increment_;
    scan.angle_increment = increment_;
    scan.scan_time = rate ? 1.0 / rate : 0.0;
    scan.range_min = p.min_range;
    scan.range_max =
        std::min(p.max_range, (first.data.size() - 1) * resolution);
    scan.ranges.assign(count, std::numeric_limits<float>::quiet_NaN());
    scan.intensities.assign(count, 0);
  }
  void add(const Spoke &s, const std::vector<int> &bins, double resolution,
           bool nearest) {
    const auto position = positive_angle(s.azimuth_angle - start_) / increment_;
    const auto cell = static_cast<size_t>(std::llround(position));
    if (std::abs(position - cell) > 0.25) {
      regular_ = false;
      return;
    }
    const auto i = cell % scan.ranges.size();
    auto &range = scan.ranges[i];
    auto &level = scan.intensities[i];
    const auto elapsed = (nanoseconds(s.header.stamp) - first_stamp_) * 1e-9;
    regular_ = regular_ && std::isnan(range) &&
               std::abs(elapsed - i * dt_) <= std::max(1e-6, dt_ * 0.1);
    if (std::isnan(range))
      range = std::numeric_limits<float>::infinity();
    if (bins.empty())
      return;
    const int best = nearest ? bins.front()
                             : *std::max_element(bins.begin(), bins.end(),
                                                 [&](int a, int b) {
                                                   return s.data[a] < s.data[b];
                                                 });
    const float distance = best * resolution;
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
};
} // namespace radar
