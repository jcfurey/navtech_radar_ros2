// SPDX-License-Identifier: Apache-2.0
#include "cloud_deskew.hpp"
#include "navtech_radar_msgs/msg/fft_sweep.hpp"
#include "navtech_radar_msgs/msg/radar_configuration.hpp"
#include "planar_products.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
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
  explicit SweepAssembler(const rclcpp::NodeOptions &options)
      : Node("navtech_fft_sweep_assembler", options) {
    frame_ = param<std::string>("output_frame", "");
    resolution_ = param("range_resolution", 0.0438);
    axis_ = {resolution_, 0};
    count_ = param("expected_spokes", 400);
    fallback_gap_ = param("fallback_max_spoke_gap_sec", 1.0);
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
    if (!(fallback_gap_ > 0) || !std::isfinite(fallback_gap_))
      throw std::invalid_argument(
          "fallback_max_spoke_gap_sec must be finite and positive");
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
    const auto fixed = param<std::string>("deskew_fixed_frame", "");
    if (fixed.find('/') == 0)
      throw std::invalid_argument(
          "deskew_fixed_frame must have no leading slash");
    if (publish_cloud_ && !fixed.empty())
      deskew_ = std::make_unique<CloudDeskew>(*this, fixed, clouds_);
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

  bool crossed_zero(const Spoke &previous, const Spoke &current,
                    int64_t gap_ns) const {
    if (current.azimuth_angle < previous.azimuth_angle - wrap_)
      return true;
    if (!config_)
      return false;

    // The Colossus FFT counter advances once per packet, modulo 65536.
    const auto steps =
        static_cast<uint16_t>(current.sweep_counter - previous.sweep_counter);
    if (steps == 0 || (current.azimuth >= previous.azimuth && steps < count_))
      return false;

    // Test the forward path that passes through zero once. A half-cell tie
    // gives no unique packet count. A known period also distinguishes this
    // path from a counter reset or a small out-of-order encoder movement.
    const int ticks =
        config_->encoder_size + current.azimuth - previous.azimuth;
    const double cells = double(ticks) * count_ / config_->encoder_size;
    if (std::abs(cells - steps) >= 0.5)
      return false;
    const auto rate = config_->expected_rotation_rate;
    return !rate || nanoseconds(previous.header.stamp) == 0 ||
           nanoseconds(current.header.stamp) == 0 ||
           std::abs(gap_ns * 1e-9 * rate -
                    double(ticks) / config_->encoder_size) < 0.5;
  }

  // The TAMU recording occasionally inverts two adjacent acquisition times
  // while both the FFT packet counter and encoder continue forward.
  bool local_stamp_inversion(const Spoke &current, int64_t gap_ns) const {
    if (pending_.size() < 2 || !config_ || !config_->expected_rotation_rate)
      return false;
    const int64_t hz = int64_t(config_->expected_rotation_rate) * count_;
    const auto &previous = *pending_.back();
    const auto earlier =
        nanoseconds(pending_[pending_.size() - 2]->header.stamp);
    const double cells = double(current.azimuth - previous.azimuth) * count_ /
                         config_->encoder_size;
    return -gap_ns <= 999999999LL / hz && earlier > 0 &&
           nanoseconds(current.header.stamp) > earlier &&
           uint16_t(current.sweep_counter - previous.sweep_counter) == 1 &&
           std::abs(cells - 1.0) < 0.5;
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
    if (config_) {
      const double encoder_angle = tau * s->azimuth / config_->encoder_size;
      if (std::abs(std::remainder(s->azimuth_angle - encoder_angle, tau)) >
          tau / (2 * config_->encoder_size) + 1e-12) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "FFT spoke angle disagrees with encoder count");
        return;
      }
      s->azimuth_angle = encoder_angle;
    }
    if (!pending_.empty()) {
      const auto &prev = *pending_.back();
      const auto gap =
          nanoseconds(s->header.stamp) - nanoseconds(prev.header.stamp);
      const int rate = config_ ? config_->expected_rotation_rate : 0;
      const bool forward_jump = rate ? gap >= (1000000000LL + rate - 1) / rate
                                     : gap * 1e-9 > fallback_gap_;
      if ((gap < 0 && !local_stamp_inversion(*s, gap)) || forward_jump) {
        pending_.clear();
        if (deskew_)
          deskew_->clear();
      } else if (s->data.size() != prev.data.size() ||
                 s->sample_bit_depth != prev.sample_bit_depth ||
                 s->header.frame_id != prev.header.frame_id ||
                 crossed_zero(prev, *s, gap))
        finish();
    }
    if (pending_.size() >= static_cast<size_t>(max_rows_)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Sweep buffer limit reached");
      pending_.clear();
    }
    pending_.push_back(std::move(s));
  }

  bool complete(double start) const {
    if (pending_.size() != static_cast<size_t>(count_))
      return false;
    std::vector<bool> scan_seen(count_), encoder_seen(config_ ? count_ : 65536);
    for (const auto &s : pending_) {
      const int encoder =
          config_ ? encoder_cell(s->azimuth, count_, config_->encoder_size)
                  : s->azimuth;
      const auto cell = scan_cell(s->azimuth_angle, start, count_);
      if (!cell || scan_seen[*cell] || encoder_seen[encoder])
        return false;
      scan_seen[*cell] = encoder_seen[encoder] = true;
    }
    return true;
  }

  void finish() {
    if (pending_.size() < static_cast<size_t>(minimum_)) {
      pending_.clear();
      return;
    }
    const auto &first = *pending_.front();
    const auto end = pending_.back()->header.stamp;
    if (!partial_ && !complete(first.azimuth_angle)) {
      pending_.clear();
      return;
    }
    std::stable_sort(
        pending_.begin(), pending_.end(),
        [](const auto &a, const auto &b) { return a->azimuth < b->azimuth; });

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
    auto sweep = std::make_unique<Sweep>();
    auto &image = sweep->image;
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
        sweep->encoder_values.push_back(s->azimuth);
        sweep->sweep_counters.push_back(s->sweep_counter);
        sweep->timestamps.push_back(s->header.stamp);
        sweep->host_stamps.push_back(s->host_stamp);
      }
    }
    if (want_image)
      images_->publish(image);
    if (want_sweep) {
      sweep->header = first.header;
      sweep->end_stamp = end;
      sweep->configuration_version = first.configuration_version;
      sweep->sweep_sequence = sequence_;
      sweep->sample_bit_depth = first.sample_bit_depth;
      sweep->range_resolution = resolution_;
      sweeps_->publish(std::move(sweep));
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
    std::vector<SpokePoints> spans;
    for (const auto &s : pending_) {
      const auto &bins = detector_.select(s->data, axis_, settings_);
      if (scan)
        scan->add(*s, bins, axis_, settings_.nearest);
      if (cloud && !bins.empty()) {
        append_points(*cloud, *s, bins, axis_, offset_, settings_.cloud_limit,
                      cloud_bins_);
        if (deskew_)
          spans.push_back({s->header.stamp, cloud->width});
      }
    }
    if (cloud) {
      if (deskew_)
        deskew_->submit(std::move(*cloud), std::move(spans));
      else
        clouds_->publish(*cloud);
    }
    if (scan) {
      const auto &msg = scan->finish();
      if (msg.range_max >= msg.range_min)
        scans_->publish(msg);
    }
  }

  std::string frame_;
  double resolution_, offset_, wrap_, fallback_gap_;
  int count_, minimum_, max_rows_, max_bins_;
  bool partial_, publish_cloud_, publish_scan_;
  uint32_t sequence_{};
  DetectionSettings settings_;
  RangeAxis axis_;
  std::unique_ptr<CloudDeskew> deskew_;
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

RCLCPP_COMPONENTS_REGISTER_NODE(radar::SweepAssembler)
