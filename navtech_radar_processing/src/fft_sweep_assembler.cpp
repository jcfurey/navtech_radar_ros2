// SPDX-License-Identifier: Apache-2.0
#include "navtech_radar_msgs/msg/fft_sweep.hpp"
#include "navtech_radar_msgs/msg/radar_configuration.hpp"
#include "planar_products.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>

namespace radar {
using Config = navtech_radar_msgs::msg::RadarConfiguration;
using Sweep = navtech_radar_msgs::msg::FftSweep;
using Image = sensor_msgs::msg::Image;

class SweepAssembler final : public rclcpp::Node {
public:
  SweepAssembler() : Node("navtech_fft_sweep_assembler") {
    frame_ = param<std::string>("output_frame", "");
    resolution_ = param("range_resolution", 0.0438);
    axis_ = {resolution_, 0};
    count_ = param("expected_spokes", 400);
    partial_ = param("publish_partial_sweeps", true);
    minimum_ = param("minimum_spokes", 2);
    max_rows_ = param("max_buffered_spokes", 2048);
    max_bins_ = param("max_range_bins", 16384);
    publish_cloud_ = param("publish_point_cloud", false);
    publish_scan_ = param("publish_scan", false);
    offset_ = param("azimuth_offset_degrees", 0.0) * (tau / 360.0);
    wrap_ = param("wrap_threshold_degrees", 180.0) * (tau / 360.0);
    const auto mode = param<std::string>("peak_selection", "threshold");
    const auto selection = param<std::string>("scan_selection", "nearest");
    settings_ = {param("min_range", 2.0),
                 param("max_range", 200.0),
                 param("cfar_threshold_offset", 3000.0),
                 param("min_peak_separation", 0.5),
                 param("intensity_threshold", 22000),
                 param("cfar_training_cells", 16),
                 param("cfar_guard_cells", 4),
                 param("max_points_per_spoke", 48),
                 mode == "cfar",
                 selection == "nearest"};
    const auto depth = param("input_queue_depth", 512);
    const auto &p = settings_;
    if (mode != "cfar" && mode != "threshold")
      throw std::invalid_argument("invalid peak_selection");
    if (selection != "nearest" && selection != "strongest")
      throw std::invalid_argument("invalid scan_selection");
    if (depth <= 0 || frame_.find('/') == 0 || !(resolution_ > 0) ||
        !std::isfinite(resolution_) || !std::isfinite(offset_) ||
        !(wrap_ > 0 && wrap_ < tau) || minimum_ < 1 || count_ < minimum_ ||
        count_ > UINT16_MAX || max_rows_ < count_ || max_bins_ < 1 ||
        max_bins_ > UINT16_MAX)
      throw std::invalid_argument(
          "invalid sweep geometry, frame, or queue limits");
    if (!(p.min_range >= 0 && p.max_range > p.min_range) ||
        !std::isfinite(p.max_range) || !std::isfinite(p.separation) ||
        p.separation < 0 || !std::isfinite(p.cfar_offset) ||
        p.cfar_offset < 0 || p.threshold < 0 || p.threshold > UINT16_MAX ||
        p.cloud_limit < 0 || p.training < 1 || p.training > 1024 ||
        p.guard < 0 || p.guard > 1024)
      throw std::invalid_argument("invalid detection settings");

    sweeps_ = create_publisher<Sweep>(
        param<std::string>("sweep_topic", "navtech_radar/fft/sweep"), 2);
    images_ = create_publisher<Image>(
        param<std::string>("image_topic", "navtech_radar/fft/image"), 2);
    clouds_ = create_publisher<Cloud>(
        param<std::string>("point_cloud_topic", "navtech_radar/point_cloud"),
        rclcpp::SensorDataQoS());
    scans_ = create_publisher<Scan>(
        param<std::string>("scan_topic", "navtech_radar/scan"),
        rclcpp::SensorDataQoS());
    configs_ = create_subscription<Config>(
        param<std::string>("configuration_topic",
                           "navtech_radar/configuration"),
        rclcpp::QoS(1).transient_local(),
        [this](Config::ConstSharedPtr c) { configure(std::move(c)); });
    spokes_ = create_subscription<Spoke>(
        param<std::string>("spoke_topic", "navtech_radar/fft/spoke"),
        rclcpp::QoS(depth).best_effort(),
        [this](Spoke::UniquePtr s) { accept(std::move(s)); });
  }

private:
  template <class T> T param(const std::string &name, T value) {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.read_only = true;
    return declare_parameter<T>(name, value, desc);
  }

  bool valid_frame(const std::string &id) const {
    return !id.empty() && id.front() != '/' && (frame_.empty() || frame_ == id);
  }

  void configure(Config::ConstSharedPtr c) {
    const auto axis = RangeAxis::from_config(*c);
    if (!axis.valid(c->range_bins) || !valid_frame(c->header.frame_id) ||
        !c->configuration_version ||
        !(std::isfinite(c->range_resolution) && c->range_resolution > 0) ||
        !c->azimuth_samples || c->azimuth_samples > c->encoder_size ||
        c->azimuth_samples > max_rows_ || !c->range_bins ||
        c->range_bins > max_bins_ ||
        (c->sample_bit_depth != 8 && c->sample_bit_depth != 16)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Ignoring invalid configuration");
      return;
    }
    auto geometry = [](const Config &msg) {
      return std::tie(msg.configuration_version, msg.range_resolution,
                      msg.azimuth_samples, msg.encoder_size, msg.range_bins,
                      msg.sample_bit_depth, msg.header.frame_id,
                      msg.expected_rotation_rate, msg.range_gain,
                      msg.range_offset);
    };
    if (config_ && geometry(*config_) == geometry(*c))
      return;
    finish();
    config_ = std::move(c);
    axis_ = axis;
    resolution_ = config_->range_resolution;
    count_ = config_->azimuth_samples;
  }

  void accept(Spoke::UniquePtr s) {
    if (!valid_frame(s->header.frame_id) || !std::isfinite(s->azimuth_angle) ||
        s->header.stamp.nanosec >= 1000000000u ||
        s->host_stamp.nanosec >= 1000000000u || s->data.empty() ||
        s->data.size() > static_cast<size_t>(max_bins_) ||
        s->configuration_version !=
            (config_ ? config_->configuration_version : 0) ||
        (s->sample_bit_depth != 8 && s->sample_bit_depth != 16) ||
        (config_ && (s->header.frame_id != config_->header.frame_id ||
                     s->data.size() != config_->range_bins ||
                     s->sample_bit_depth != config_->sample_bit_depth ||
                     s->azimuth >= config_->encoder_size)) ||
        (s->sample_bit_depth == 8 &&
         *std::max_element(s->data.begin(), s->data.end()) > 255)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Ignoring incompatible FFT spoke");
      return;
    }
    s->azimuth_angle = positive_angle(s->azimuth_angle);
    if (!pending_.empty()) {
      const auto &prev = *pending_.back();
      if (nanoseconds(s->header.stamp) < nanoseconds(prev.header.stamp))
        pending_.clear();
      else if (s->data.size() != prev.data.size() ||
               s->sample_bit_depth != prev.sample_bit_depth ||
               s->header.frame_id != prev.header.frame_id ||
               s->azimuth_angle < prev.azimuth_angle - wrap_)
        finish();
    }
    if (pending_.size() >= static_cast<size_t>(max_rows_)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Sweep buffer limit reached");
      pending_.clear();
    }
    pending_.push_back(std::move(s));
  }

  void finish() {
    if (pending_.size() < static_cast<size_t>(minimum_)) {
      pending_.clear();
      return;
    }
    const auto &first = *pending_.front();
    const auto end = pending_.back()->header.stamp;
    std::stable_sort(
        pending_.begin(), pending_.end(),
        [](const auto &a, const auto &b) { return a->azimuth < b->azimuth; });
    if (!partial_ && (pending_.size() != static_cast<size_t>(count_) ||
                      std::adjacent_find(pending_.begin(), pending_.end(),
                                         [](const auto &a, const auto &b) {
                                           return a->azimuth == b->azimuth;
                                         }) != pending_.end())) {
      pending_.clear();
      return;
    }

    // DDS volatile outputs have no history for future subscribers. Avoid
    // materializing a multi-megabyte image for a scan-only consumer.
    const bool want_sweep = sweeps_->get_subscription_count() > 0;
    const bool want_image = images_->get_subscription_count() > 0;
    if (want_sweep || want_image)
      publish_raw(first, end, want_sweep, want_image);
    if (publish_scan_ || publish_cloud_)
      publish_detections(first);
    ++sequence_;
    pending_.clear();
  }

  void publish_raw(const Spoke &first, const builtin_interfaces::msg::Time &end,
                   bool want_sweep, bool want_image) {
    Sweep sweep;
    auto &image = sweep.image;
    image.header = first.header;
    image.width = first.data.size();
    image.height = pending_.size();
    image.encoding = "mono16";
    image.step = image.width * 2;
    image.data.resize(static_cast<size_t>(image.step) * image.height);
    auto *out = image.data.data();
    for (const auto &s : pending_) {
      if (!big_endian())
        std::memcpy(out, s->data.data(), image.step);
      else
        for (size_t i = 0; i < s->data.size(); ++i) {
          out[2 * i] = s->data[i] & 255;
          out[2 * i + 1] = s->data[i] >> 8;
        }
      out += image.step;
      if (want_sweep) {
        sweep.encoder_values.push_back(s->azimuth);
        sweep.sweep_counters.push_back(s->sweep_counter);
        sweep.timestamps.push_back(s->header.stamp);
        sweep.host_stamps.push_back(s->host_stamp);
      }
    }
    if (want_image)
      images_->publish(image);
    if (want_sweep) {
      sweep.header = first.header;
      sweep.end_stamp = end;
      sweep.configuration_version = first.configuration_version;
      sweep.sweep_sequence = sequence_;
      sweep.sample_bit_depth = first.sample_bit_depth;
      sweep.range_resolution = resolution_;
      sweeps_->publish(sweep);
    }
  }

  void publish_detections(const Spoke &first) {
    std::optional<ScanBuilder> scan;
    std::optional<Cloud> cloud;
    if (publish_scan_ && scans_->get_subscription_count())
      scan.emplace(first, count_, axis_,
                   config_ ? config_->expected_rotation_rate : 0, offset_,
                   settings_);
    if (publish_cloud_ && clouds_->get_subscription_count())
      cloud = empty_cloud(first.header);
    if (!scan && !cloud)
      return;
    for (const auto &s : pending_) {
      const auto &bins = detector_.select(s->data, axis_, settings_);
      if (scan)
        scan->add(*s, bins, axis_, settings_.nearest);
      if (cloud)
        append_points(*cloud, *s, bins, axis_, offset_, settings_.cloud_limit,
                      cloud_bins_);
    }
    if (cloud)
      clouds_->publish(*cloud);
    if (scan) {
      const auto &msg = scan->finish();
      if (msg.range_max >= msg.range_min)
        scans_->publish(msg);
    }
  }

  std::string frame_;
  double resolution_, offset_, wrap_;
  int count_, minimum_, max_rows_, max_bins_;
  bool partial_, publish_cloud_, publish_scan_;
  uint32_t sequence_{};
  DetectionSettings settings_;
  RangeAxis axis_;
  PeakDetector detector_;
  std::vector<int> cloud_bins_;
  Config::ConstSharedPtr config_;
  std::vector<Spoke::UniquePtr> pending_;
  rclcpp::Publisher<Sweep>::SharedPtr sweeps_;
  rclcpp::Publisher<Image>::SharedPtr images_;
  rclcpp::Publisher<Cloud>::SharedPtr clouds_;
  rclcpp::Publisher<Scan>::SharedPtr scans_;
  rclcpp::Subscription<Config>::SharedPtr configs_;
  rclcpp::Subscription<Spoke>::SharedPtr spokes_;
};
} // namespace radar

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<radar::SweepAssembler>());
  rclcpp::shutdown();
}
