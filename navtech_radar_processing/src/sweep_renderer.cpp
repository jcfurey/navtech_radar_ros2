// SPDX-License-Identifier: Apache-2.0
#include "sweep_renderer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace radar {
namespace {
constexpr double radians = CV_PI / 180.0;
constexpr double turn = 2 * CV_PI;

double wrap(double x, double period) {
  const double r = std::fmod(x, period);
  return r < 0 ? r + period : std::abs(r);
}

int nearest(double x) { return static_cast<int>(std::nearbyint(x)); }

cv::Mat3b canvas(Image &image, const Sweep &s, int height, int width) {
  image.header = s.header;
  image.encoding = "bgr8";
  image.height = height;
  image.width = width;
  image.step = width * 3;
  image.data.resize(static_cast<size_t>(height) * image.step);
  return cv::Mat3b(height, width,
                   reinterpret_cast<cv::Vec3b *>(image.data.data()));
}

void text(cv::Mat3b &image, const std::string &label, cv::Point at) {
  cv::putText(image, label, at, cv::FONT_HERSHEY_SIMPLEX, 0.45,
              cv::Scalar::all(0), 3, cv::LINE_AA);
  cv::putText(image, label, at, cv::FONT_HERSHEY_SIMPLEX, 0.45,
              cv::Scalar::all(220), 1, cv::LINE_AA);
}

std::string heading_label(const DisplayHeading &heading) {
  if (!heading.yaw)
    return "Vehicle yaw: unavailable (TF)";
  return cv::format("Vehicle yaw in %s: %.1f deg (+z)", heading.frame.c_str(),
                    wrap(*heading.yaw / radians, 360.0));
}

void annotate_polar(cv::Mat3b &out, const Sweep &s, const Config &c, int width,
                    double minimum, double maximum, double low, double high,
                    double offset, const std::string &coverage,
                    const DisplayHeading &heading) {
  const int height = c.azimuth_samples;
  text(out, "Bearing (+z, deg) / range (m) | " + s.header.frame_id, {48, 20});
  text(out, heading_label(heading), {48, 40});
  for (int a : {0, 90, 180, 270}) {
    const int row = 52 + nearest(a * height / 360.0);
    text(out, cv::format("%.0f", wrap(offset / radians - a, 360.0)),
         {5, row + 5});
  }
  if (heading.robot_bearing) {
    const int row = 52 + nearest(wrap(offset - *heading.robot_bearing, turn) *
                                 height / turn) %
                             height;
    cv::line(out, {48, row}, {width + 47, row}, {255, 220, 0}, 1);
    text(out, "vehicle forward", {54, std::min(row + 16, height + 48)});
  }
  for (double f : {0., 0.25, 0.5, 0.75, 1.}) {
    const int col = 48 + nearest(f * (width - 1));
    text(out, cv::format("%.0f", minimum + f * (maximum - minimum)),
         {std::min(col, width + 36), height + 70});
  }
  text(out, cv::format("FFT level: %.0f (dark) - %.0f (bright)", low, high),
       {48, height + 90});
  text(out, "Sweep " + std::to_string(s.sweep_sequence) + " | " + coverage,
       {48, height + 112});
  text(out,
       cv::format("%d Hz | %.2f cm/bin | raw FFT levels",
                  int(c.expected_rotation_rate),
                  RangeAxis::from_config(c).spacing * 100),
       {48, height + 134});
}

void annotate_cartesian(cv::Mat3b &out, const Sweep &s, const Config &c,
                        double extent, const std::string &coverage,
                        const DisplayHeading &heading) {
  const int center = (out.rows - 1) / 2;
  const cv::Point origin{center, center};
  for (double f : {1.0 / 3, 2.0 / 3, 1.0}) {
    const int radius = std::max(1, nearest(center * f));
    cv::circle(out, origin, radius, cv::Scalar::all(65), 1, cv::LINE_AA);
    text(out, cv::format("%.0f m", extent * f),
         {center + 5, std::max(35, center - radius + 15)});
  }
  for (int a = 0; a < 360; a += 45) {
    const int x = nearest(center - center * 0.88 * std::sin(a * radians));
    const int y = nearest(center - center * 0.88 * std::cos(a * radians));
    text(out, std::to_string(a), {x - 12, y + 5});
  }
  cv::drawMarker(out, origin, cv::Scalar::all(180), cv::MARKER_CROSS, 16, 1,
                 cv::LINE_AA);
  text(out, "+x", {center + 5, std::max(58, center - 40)});
  text(out, "+y", {std::max(5, center - 60), center - 5});
  if (heading.robot_bearing) {
    const int length = std::max(16, nearest(center * 0.2));
    const cv::Point tip{
        nearest(center - length * std::sin(*heading.robot_bearing)),
        nearest(center - length * std::cos(*heading.robot_bearing))};
    cv::arrowedLine(out, origin, tip, {255, 220, 0}, 2, cv::LINE_AA);
    text(out, "vehicle forward", tip + cv::Point(6, -6));
  }
  text(out, "Radar plan view | " + s.header.frame_id + " | +x up, +y left",
       {12, 20});
  text(out, heading_label(heading), {12, 40});
  text(out, coverage + cv::format(" | %d Hz", int(c.expected_rotation_rate)),
       {12, out.rows - 12});
}

// The histogram contains cumulative counts of pooled samples from observed
// rows.
double percentile(const std::array<uint32_t, 65536> &counts, double percent) {
  const double rank = (percent / 100.0) * (counts.back() - 1);
  auto value_at = [&](uint32_t n) {
    return std::upper_bound(counts.begin(), counts.end(), n) - counts.begin();
  };
  const int lo = value_at(static_cast<uint32_t>(rank));
  const int hi = value_at(static_cast<uint32_t>(std::ceil(rank)));
  return lo + (rank - std::floor(rank)) * (hi - lo);
}
} // namespace

SweepRenderer::SweepRenderer(DisplaySettings settings)
    : settings_(settings), offset_(settings.azimuth_offset_degrees * radians) {
  const auto &p = settings_;
  if (!(p.min_range >= 0 && p.min_range < p.max_range) ||
      !std::isfinite(p.max_range) || p.image_width < 64 ||
      p.image_width > 4096 || p.cartesian_size < 64 ||
      p.cartesian_size > 2048 ||
      !(0 <= p.lower_percentile && p.lower_percentile < p.upper_percentile &&
        p.upper_percentile <= 100) ||
      !(0 < p.contrast_alpha && p.contrast_alpha <= 1) ||
      !(0 < p.gamma && p.gamma <= 5) || !std::isfinite(offset_))
    throw std::invalid_argument("Invalid sweep display parameters");
  cv::Mat1b levels(256, 1);
  for (int i = 0; i < 256; ++i)
    levels(i) = i;
  cv::applyColorMap(levels, palette_, cv::COLORMAP_INFERNO);
  palette_(0) = {0, 0, 0};
}

bool SweepRenderer::valid_configuration(const Config &c) {
  return c.configuration_version && !c.header.frame_id.empty() &&
         c.azimuth_samples > 0 && c.azimuth_samples <= 2048 &&
         c.azimuth_samples <= c.encoder_size && c.range_bins > 0 &&
         c.range_bins <= 16384 && std::isfinite(c.range_resolution) &&
         c.range_resolution > 0 &&
         RangeAxis::from_config(c).valid(c.range_bins);
}

void SweepRenderer::update_projection(int rows, int cols, int first, int stride,
                                      const RangeAxis &range_axis,
                                      double extent) {
  const Projection key{
      rows, cols, first, stride, range_axis.spacing, range_axis.offset, extent};
  if (projection_ == key)
    return;
  const int size = settings_.cartesian_size;
  const double center = (size - 1) / 2.0;
  std::vector<double> axis(size);
  for (int i = 0; i < size; ++i) {
    const double d = center - i;
    axis[i] = extent <= std::numeric_limits<double>::max() / center
                  ? d * extent / center
                  : (d / center) * extent;
  }
  indices_.create(size, size);
  for (int y = 0; y < size; ++y) {
    auto *out = indices_.ptr<int>(y);
    for (int x = 0; x < size; ++x) {
      const double range = std::hypot(axis[y], axis[x]);
      const double bin =
          (range - range_axis.offset) / range_axis.spacing - first;
      if (range < settings_.min_range || range > extent || bin < 0) {
        out[x] = -1;
        continue;
      }
      const double angle = wrap(offset_ - std::atan2(axis[x], axis[y]), turn);
      const int row = int(std::floor(angle * rows / turn + 0.5)) % rows;
      out[x] = row * cols +
               static_cast<int>(std::min(bin / stride, double(cols - 1)));
    }
  }
  projection_ = key;
}

void SweepRenderer::update_colors(const cv::Mat1w &grid,
                                  const std::vector<bool> &present,
                                  const Sweep &s, const Config &c) {
  const int rows = grid.rows, cols = grid.cols;
  histogram_.fill(0);
  for (int row = 0; row < rows; ++row) {
    if (!present[row])
      continue;
    const auto *levels = grid.ptr<uint16_t>(row);
    for (int col = 0; col < cols; ++col)
      ++histogram_[levels[col]];
  }
  std::partial_sum(histogram_.begin(), histogram_.end(), histogram_.begin());
  const double low = percentile(histogram_, settings_.lower_percentile);
  const double high = percentile(histogram_, settings_.upper_percentile);
  const Geometry epoch{c.configuration_version,
                       c.header.frame_id,
                       c.encoder_size,
                       rows,
                       c.range_bins,
                       s.range_resolution,
                       c.range_gain,
                       c.range_offset};
  const int64_t stamp =
      int64_t(s.header.stamp.sec) * 1000000000 + s.header.stamp.nanosec;
  if (epoch_ != epoch || stamp < stamp_) {
    low_ = low;
    high_ = high;
  } else {
    const double a = settings_.contrast_alpha;
    low_ = (1 - a) * low_ + a * low;
    high_ = (1 - a) * high_ + a * high;
  }
  epoch_ = epoch;
  stamp_ = stamp;
  const double spread = std::max(high_ - low_, 1.0);
  colors_[0] = palette_(0);
  for (size_t i = 1; i < colors_.size(); ++i) {
    if (histogram_[i] == histogram_[i - 1])
      continue;
    double level = std::clamp((i - low_) / spread, 0.0, 1.0);
    if (settings_.gamma != 1.0)
      level = std::pow(level, settings_.gamma);
    colors_[i] = palette_(nearest(level * 255));
  }
}

void SweepRenderer::render(const Sweep &s, const Config &c,
                           const DisplayHeading &heading, Image *polar,
                           Image *cartesian) {
  const auto &image = s.image;
  const double res = s.range_resolution;
  if (!valid_configuration(c) ||
      s.configuration_version != c.configuration_version ||
      s.header.frame_id != c.header.frame_id || res != c.range_resolution ||
      image.encoding != "mono16" || !image.height || image.height > 2048 ||
      image.width != c.range_bins || image.step < 2 * image.width ||
      image.step % 2 ||
      image.data.size() != size_t(image.height) * image.step ||
      s.encoder_values.size() != image.height ||
      std::any_of(
          s.encoder_values.begin(), s.encoder_values.end(),
          [&](auto v) { return v >= c.encoder_size; }))
    throw std::invalid_argument(
        "Sweep image or encoder metadata does not match configuration");
  const int width = image.width;
  const auto range = RangeAxis::from_config(c);
  const auto [first, last] =
      range.window(width, settings_.min_range, settings_.max_range);
  if (first >= last)
    throw std::invalid_argument("Display range does not overlap the FFT bins");
  const int stride =
      (last - first + settings_.image_width - 1) / settings_.image_width;
  const int cols = (last - first + stride - 1) / stride;
  const int rows = c.azimuth_samples;
  cv::Mat1w grid(rows, cols, uint16_t(0));
  std::vector<bool> present(rows);
  // Pool directly into the encoder row so duplicate bearings keep the strongest
  // bin.
  for (size_t i = 0; i < image.height; ++i) {
    const int encoder = s.encoder_values[i];
    const int row = encoder_cell(encoder, rows, c.encoder_size);
    present[row] = true;
    auto *pooled = grid.ptr<uint16_t>(row);
    const auto *data = image.data.data() + i * image.step;
    for (int col = 0, bin = first; col < cols; ++col) {
      uint16_t maximum = pooled[col];
      const int end = std::min(bin + stride, last);
      for (; bin < end; ++bin) {
        const auto *p = data + 2 * bin;
        const uint16_t level =
            image.is_bigendian ? (p[0] << 8 | p[1]) : (p[1] << 8 | p[0]);
        maximum = std::max(maximum, level);
      }
      pooled[col] = maximum;
    }
  }
  update_colors(grid, present, s, c);
  const int observed = std::count(present.begin(), present.end(), true);
  const auto coverage =
      cv::format("%d/%d bearings | %d missing | %d duplicates", observed, rows,
                 rows - observed, int(image.height) - observed);
  if (polar) {
    const int top = settings_.annotate ? 52 : 0,
              left = settings_.annotate ? 48 : 0;
    auto out = canvas(*polar, s, rows + (settings_.annotate ? 150 : 0),
                      cols + (settings_.annotate ? 66 : 0));
    for (int row = 0; row < rows; ++row) {
      auto *dest = out.ptr<cv::Vec3b>(row + top) + left;
      const auto *levels = grid.ptr<uint16_t>(row);
      for (int col = 0; col < cols; ++col)
        dest[col] = colors_[levels[col]];
    }
    if (settings_.annotate)
      annotate_polar(out, s, c, cols, range.metres(first),
                     range.metres(last - 1), low_, high_, offset_, coverage,
                     heading);
  }
  if (cartesian) {
    const double extent =
        std::min(settings_.max_range, range.metres(width - 1));
    update_projection(rows, cols, first, stride, range, extent);
    auto out = canvas(*cartesian, s, indices_.rows, indices_.cols);
    const auto *levels = grid.ptr<uint16_t>();
    for (int y = 0; y < out.rows; ++y) {
      auto *dest = out.ptr<cv::Vec3b>(y);
      const auto *index = indices_.ptr<int>(y);
      for (int x = 0; x < out.cols; ++x)
        dest[x] = index[x] < 0 ? cv::Vec3b(0, 0, 0) : colors_[levels[index[x]]];
    }
    if (settings_.annotate)
      annotate_cartesian(out, s, c, extent, coverage, heading);
  }
}
} // namespace radar
