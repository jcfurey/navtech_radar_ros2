// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "navtech_radar_msgs/msg/fft_sweep.hpp"
#include "navtech_radar_msgs/msg/radar_configuration.hpp"
#include <opencv2/core.hpp>

namespace radar {
using Sweep = navtech_radar_msgs::msg::FftSweep;
using Config = navtech_radar_msgs::msg::RadarConfiguration;
using Image = sensor_msgs::msg::Image;

struct DisplaySettings {
  double min_range, max_range;
  int64_t image_width, cartesian_size;
  double lower_percentile, upper_percentile, contrast_alpha, gamma;
  bool annotate;
  double azimuth_offset_degrees;
};

struct DisplayHeading {
  std::optional<double> robot_bearing, yaw;
  std::string frame;
};

class SweepRenderer {
public:
  explicit SweepRenderer(DisplaySettings settings);
  static bool valid_configuration(const Config &c);
  void render(const Sweep &sweep, const Config &config,
              const DisplayHeading &heading, Image *polar, Image *cartesian);

private:
  void update_colors(const cv::Mat1w &grid, const std::vector<bool> &present,
                     const Sweep &sweep, const Config &config);
  void update_projection(int rows, int cols, int first, int stride,
                         double resolution, double extent);
  DisplaySettings settings_;
  double offset_;
  cv::Mat3b palette_;
  std::array<uint32_t, 65536> histogram_{};
  std::array<cv::Vec3b, 65536> colors_{};
  using Geometry = std::tuple<uint32_t, std::string, int, int, int, double>;
  std::optional<Geometry> epoch_;
  int64_t stamp_{};
  double low_{}, high_{};
  using Projection = std::tuple<int, int, int, int, double, double>;
  std::optional<Projection> projection_;
  cv::Mat1i indices_;
};
} // namespace radar
