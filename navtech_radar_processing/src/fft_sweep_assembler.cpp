// SPDX-License-Identifier: Apache-2.0
#include "cloud_deskew.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "navtech_radar_msgs/msg/fft_sweep.hpp"
#include "navtech_radar_msgs/msg/radar_configuration.hpp"
#include "navtech_radar_msgs/msg/recorded_rotation.hpp"
#include "planar_products.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <deque>
#include <memory>
#include <omp.h>
#include <optional>
#include <string>
#include <tuple>
#include <utility>

namespace radar {
using Config = navtech_radar_msgs::msg::RadarConfiguration;
using Recording = navtech_radar_msgs::msg::RecordedRotation;
using Sweep = navtech_radar_msgs::msg::FftSweep;
using Image = sensor_msgs::msg::Image;
using Diagnostics = diagnostic_msgs::msg::DiagnosticArray;

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
    const auto depth = param("input_queue_depth", 1024);
    bag_spokes_ = param<int64_t>("bag_spoke_count", 0);
    if (bag_spokes_ < 0)
      throw std::invalid_argument("negative bag_spoke_count");
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
    const int threads =
        param("detection_threads", std::min(8, omp_get_max_threads()));
    if (threads < 1 || threads > 32)
      throw std::invalid_argument("detection_threads must be between 1 and 32");
    detectors_.resize(threads);
    settings_8bit_ = settings_;
    settings_8bit_.threshold = param("recorded_threshold_8bit", 24);
    settings_8bit_.cfar_offset = param("recorded_cfar_offset_8bit", 12.0);
    if (settings_8bit_.threshold < 0 || settings_8bit_.threshold > 255 ||
        !std::isfinite(settings_8bit_.cfar_offset) ||
        settings_8bit_.cfar_offset < 0)
      throw std::invalid_argument("invalid 8-bit detector settings");
    diagnostics_ = create_publisher<Diagnostics>("diagnostics", 10);
    report_timer_ =
        create_wall_timer(std::chrono::seconds(1), [this] { report(); });
    if (param("recorded_input", false)) {
      config_output_ = create_publisher<Config>(
          "navtech_radar/configuration", rclcpp::QoS(1).transient_local());
      spoke_output_ = create_publisher<Spoke>("navtech_radar/fft/spoke",
                                              rclcpp::SensorDataQoS());
      recordings_ = create_subscription<Recording>(
          "navtech_radar/recording/rotation", 4,
          [this](Recording::ConstSharedPtr rotation) { replay(*rotation); });
      return;
    }
    bag_queue_limit_ = depth;
    configs_ = create_subscription<Config>(
        param<std::string>("configuration_topic",
                           "navtech_radar/configuration"),
        rclcpp::QoS(bag_spokes_ ? depth : 1).transient_local(),
        [this](Config::ConstSharedPtr c) {
          if (!bag_spokes_) {
            configure(std::move(c));
            return;
          }
          if (bag_configs_.size() == bag_queue_limit_)
            bag_configs_.pop_front();
          bag_configs_.push_back(std::move(c));
          drain_bag();
        });
    auto qos = rclcpp::QoS(depth);
    if (!bag_spokes_)
      qos.best_effort();
    spokes_ = create_subscription<Spoke>(
        param<std::string>("spoke_topic", "navtech_radar/fft/spoke"), qos,
        [this](Spoke::UniquePtr s) {
          ++received_spokes_;
          if (!bag_spokes_) {
            accept(std::move(s));
            return;
          }
          if (bag_queue_.size() == bag_queue_limit_) {
            ++rejected_spokes_;
            bag_queue_.pop_front();
          }
          bag_queue_.push_back(std::move(s));
          last_bag_input_ = std::chrono::steady_clock::now();
          drain_bag();
        });
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

  // DDS preserves order within each topic. A configuration can arrive either
  // side of its first spoke. Apply it in spoke order, including at EOF.
  void drain_bag(bool player_gone = false) {
    while (!bag_queue_.empty()) {
      const auto version = bag_queue_.front()->configuration_version;
      if (version && (!config_ || config_->configuration_version != version)) {
        const auto cfg =
            std::find_if(bag_configs_.rbegin(), bag_configs_.rend(),
                         [version](const auto &c) {
                           return c->configuration_version == version;
                         });
        if (cfg != bag_configs_.rend())
          configure(*cfg);
        else if (!player_gone)
          return;
      }
      auto s = std::move(bag_queue_.front());
      bag_queue_.pop_front();
      accept(std::move(s));
    }
    if (player_gone || received_spokes_ == static_cast<uint64_t>(bag_spokes_))
      finish();
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
    if (config_output_)
      config_output_->publish(*config_);
    axis_ = axis;
    resolution_ = config_->range_resolution;
    count_ = config_->azimuth_samples;
  }

  // File readers already know precisely where a frame ends, including at EOF.
  // Keep its acquisition order until finish() applies the normal encoder sort.
  void replay(const Recording &r) {
    const auto &img = r.image;
    const auto &cfg = r.configuration;
    const size_t n = img.height;
    const bool eight = cfg.sample_bit_depth == 8;
    auto size_ok = [n](const auto &v) { return v.empty() || v.size() == n; };
    auto times_ok = [&](const auto &v) {
      return size_ok(v) && std::all_of(v.begin(), v.end(), [](uint64_t t) {
               return t / 1000000000ULL <= INT32_MAX;
             });
    };
    if (!n || n > size_t(max_rows_) || !img.width ||
        img.width != cfg.range_bins || img.width > size_t(max_bins_) ||
        !cfg.encoder_size || !cfg.azimuth_samples ||
        cfg.azimuth_samples > cfg.encoder_size ||
        cfg.azimuth_samples > max_rows_ || !cfg.configuration_version ||
        !(std::isfinite(cfg.range_resolution) && cfg.range_resolution > 0) ||
        !RangeAxis::from_config(cfg).valid(cfg.range_bins) ||
        !valid_frame(img.header.frame_id) ||
        cfg.header.frame_id != img.header.frame_id ||
        img.header.stamp.nanosec >= 1000000000u ||
        !(eight || cfg.sample_bit_depth == 16) ||
        img.encoding != (eight ? "mono8" : "mono16") ||
        img.step != img.width * (eight ? 1 : 2) ||
        img.data.size() != n * img.step || r.encoder_values.size() != n ||
        !size_ok(r.sweep_counters) || !times_ok(r.sensor_ns) ||
        !times_ok(r.receipt_ns) || !size_ok(r.row_flags) ||
        *std::max_element(r.encoder_values.begin(), r.encoder_values.end()) >=
            cfg.encoder_size) {
      RCLCPP_WARN(get_logger(), "Rejected inconsistent recorded rotation");
      return;
    }
    configure(std::make_shared<Config>(cfg));
    const bool send_spokes = spoke_output_->get_subscription_count() > 0;
    bool used_frame_stamp = false;
    for (size_t i = 0; i < n; ++i) {
      if (!r.row_flags.empty() && r.source_format != "boreas_rt" &&
          !r.row_flags[i]) {
        ++rejected_spokes_;
        continue;
      }
      auto s = std::make_unique<Spoke>();
      s->header = img.header;
      const uint64_t sensor = r.sensor_ns.empty() ? 0 : r.sensor_ns[i];
      const uint64_t receipt = r.receipt_ns.empty() ? 0 : r.receipt_ns[i];
      used_frame_stamp |= !sensor && !receipt;
      if (receipt)
        s->host_stamp = rclcpp::Time(static_cast<int64_t>(receipt));
      if (sensor || receipt)
        s->header.stamp =
            rclcpp::Time(static_cast<int64_t>(sensor ? sensor : receipt));
      s->ntp_seconds = sensor / 1000000000ULL;
      s->ntp_split_seconds = sensor % 1000000000ULL;
      s->configuration_version = cfg.configuration_version;
      s->sample_bit_depth = cfg.sample_bit_depth;
      s->sweep_counter = r.sweep_counters.empty() ? 0 : r.sweep_counters[i];
      s->azimuth = r.encoder_values[i];
      s->azimuth_angle = tau * s->azimuth / cfg.encoder_size;
      const auto *row = img.data.data() + i * img.step;
      if (eight)
        s->data.assign(row, row + img.width);
      else {
        s->data.resize(img.width);
        if (bool(img.is_bigendian) == big_endian())
          std::memcpy(s->data.data(), row, img.step);
        else
          for (size_t j = 0; j < img.width; ++j)
            s->data[j] = img.is_bigendian ? row[2 * j] * 256 + row[2 * j + 1]
                                          : row[2 * j + 1] * 256 + row[2 * j];
      }
      if (send_spokes)
        spoke_output_->publish(*s);
      pending_.push_back(std::move(s));
    }
    received_spokes_ += n;
    finish(used_frame_stamp);
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
      ++rejected_spokes_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Ignoring incompatible FFT spoke");
      return;
    }
    s->azimuth_angle = positive_angle(s->azimuth_angle);
    if (config_) {
      const double encoder_angle = tau * s->azimuth / config_->encoder_size;
      if (std::abs(std::remainder(s->azimuth_angle - encoder_angle, tau)) >
          tau / (2 * config_->encoder_size) + 1e-12) {
        ++rejected_spokes_;
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
      // Frame-timed PNG recordings carry the same acquisition stamp on each
      // row. A change of frame stamp can span a full turn or a dataset gap.
      if (bag_spokes_ && gap &&
          (pending_.size() > 1 || forward_jump || gap < 0) &&
          !prev.ntp_seconds && !prev.ntp_split_seconds &&
          !nanoseconds(prev.host_stamp) &&
          nanoseconds(pending_.front()->header.stamp) ==
              nanoseconds(prev.header.stamp) &&
          same_stamps()) {
        finish();
      } else if ((gap < 0 && !local_stamp_inversion(*s, gap)) || forward_jump) {
        discarded_spokes_ += pending_.size();
        ++time_resets_;
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
      discarded_spokes_ += pending_.size();
      ++buffer_resets_;
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

  bool same_stamps() const {
    const auto stamp = pending_.front()->header.stamp;
    return std::all_of(pending_.begin(), pending_.end(),
                       [&](const auto &s) { return s->header.stamp == stamp; });
  }

  void report() {
    if (bag_spokes_ && (!bag_queue_.empty() || !pending_.empty()) &&
        std::chrono::steady_clock::now() - last_bag_input_ >
            std::chrono::seconds(1) &&
        spokes_->get_publisher_count() == 0)
      drain_bag(true);
    Diagnostics msg;
    msg.header.stamp = now();
    auto &s = msg.status.emplace_back();
    s.name = std::string(get_fully_qualified_name()) + ": sweep assembly";
    s.hardware_id = config_ ? config_->header.frame_id : "navtech";
    const auto losses = missing_cells_ + rejected_spokes_ + discarded_spokes_;
    const bool timing_warning =
        deskew_ && timing_ != "per-spoke" && timing_ != "unknown";
    s.level = timing_warning || losses != previous_losses_ ? s.WARN : s.OK;
    s.message = timing_warning ? "Deskew requires per-spoke timestamps"
                : losses != previous_losses_
                    ? "Incomplete or rejected radar input"
                    : "Sweep assembly";
    previous_losses_ = losses;
    auto add = [&](const char *key, const std::string &value) {
      auto &kv = s.values.emplace_back();
      kv.key = key;
      kv.value = value;
    };
    add("received_spokes", std::to_string(received_spokes_));
    if (bag_spokes_)
      add("bag_spoke_count", std::to_string(bag_spokes_));
    add("rejected_spokes", std::to_string(rejected_spokes_));
    add("missing_bearing_cells", std::to_string(missing_cells_));
    add("buffer_resets", std::to_string(buffer_resets_));
    add("time_resets", std::to_string(time_resets_));
    add("discarded_buffered_spokes", std::to_string(discarded_spokes_));
    add("sweep_timing", timing_);
    add("deskew_skipped_timing", std::to_string(deskew_skipped_));
    diagnostics_->publish(msg);
  }

  void finish(bool used_frame_stamp = false) {
    if (pending_.empty())
      return;
    std::vector<bool> seen(count_);
    for (const auto &s : pending_) {
      const int cell =
          config_ ? encoder_cell(s->azimuth, count_, config_->encoder_size)
                  : int(std::llround(s->azimuth_angle * count_ / tau)) % count_;
      seen[cell] = true;
    }
    missing_cells_ += std::count(seen.begin(), seen.end(), false);
    // Our file publisher leaves both exact-time fields unset on a row that
    // fell back to the frame time. Old independent publishers may leave all
    // auxiliary fields unset even when their header times are distinct.
    const auto has_row_time = [](const auto &s) {
      return s->ntp_seconds || s->ntp_split_seconds ||
             nanoseconds(s->host_stamp);
    };
    const bool mixed =
        bag_spokes_ &&
        !std::all_of(pending_.begin(), pending_.end(), has_row_time) &&
        std::any_of(pending_.begin(), pending_.end(), has_row_time);
    if (used_frame_stamp || mixed)
      timing_ = "frame timestamp fallback";
    else if (std::any_of(pending_.begin(), pending_.end(), [](const auto &s) {
               return nanoseconds(s->header.stamp) <= 0;
             }))
      timing_ = "missing row timestamps";
    else if (same_stamps() &&
             (pending_.size() > 1 ||
              (bag_spokes_ && !has_row_time(pending_.front()))))
      timing_ = "indistinguishable row timestamps";
    else
      timing_ = "per-spoke";
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
    if (publish_cloud_ && clouds_->get_subscription_count()) {
      if (deskew_ && timing_ != "per-spoke") {
        deskew_->clear();
        ++deskew_skipped_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Skipping deskewed cloud: %s", timing_.c_str());
      } else
        cloud = empty_cloud(first.header);
    }
    if (!scan && !cloud)
      return;
    std::vector<SpokePoints> spans;
    // A detector owns its CFAR scratch. Work across independent rows, then
    // apply duplicate-ray selection and append points in original encoder
    // order. Bound temporary bin lists even for large, dense rotations.
    constexpr size_t batch = 128;
    for (size_t start = 0; start < pending_.size(); start += batch) {
      int rows = std::min(batch, pending_.size() - start);
      selected_.resize(rows);
#pragma omp parallel for schedule(static)                                      \
    num_threads(detectors_.size()) if (rows >= 32 && detectors_.size() > 1)
      for (int i = 0; i < rows; ++i) {
        const auto &s = *pending_[start + i];
        detectors_[omp_get_thread_num()].select(
            s.data, axis_, s.sample_bit_depth == 8 ? settings_8bit_ : settings_,
            scan.has_value(), cloud.has_value(), selected_[i]);
      }
      for (int i = 0; i < rows; ++i) {
        const auto &s = pending_[start + i];
        const auto &detection = selected_[i];
        if (scan)
          scan->add(*s, detection.scan_bin, axis_, settings_.nearest);
        if (cloud && !detection.cloud_bins.empty()) {
          append_points(*cloud, *s, detection.cloud_bins, axis_, offset_);
          if (deskew_)
            spans.push_back({s->header.stamp, cloud->width});
        }
      }
    }
    if (cloud) {
      if (deskew_)
        deskew_->submit(std::move(*cloud), std::move(spans));
      else
        clouds_->publish(*cloud);
    }
    if (scan) {
      const auto &msg = scan->finish(timing_ == "per-spoke");
      if (msg.range_max >= msg.range_min)
        scans_->publish(msg);
    }
  }

  std::string frame_;
  double resolution_, offset_, wrap_, fallback_gap_;
  int count_, minimum_, max_rows_, max_bins_;
  bool partial_, publish_cloud_, publish_scan_;
  uint32_t sequence_{};
  DetectionSettings settings_, settings_8bit_;
  int64_t bag_spokes_{};
  size_t bag_queue_limit_{};
  std::deque<Config::ConstSharedPtr> bag_configs_;
  std::deque<Spoke::UniquePtr> bag_queue_;
  std::chrono::steady_clock::time_point last_bag_input_{};
  uint64_t received_spokes_{}, rejected_spokes_{}, missing_cells_{},
      buffer_resets_{}, time_resets_{}, discarded_spokes_{}, deskew_skipped_{},
      previous_losses_{};
  std::string timing_{"unknown"};
  rclcpp::Publisher<Diagnostics>::SharedPtr diagnostics_;
  rclcpp::TimerBase::SharedPtr report_timer_;
  rclcpp::Publisher<Config>::SharedPtr config_output_;
  rclcpp::Publisher<Spoke>::SharedPtr spoke_output_;
  rclcpp::Subscription<Recording>::SharedPtr recordings_;
  RangeAxis axis_;
  std::unique_ptr<CloudDeskew> deskew_;
  std::vector<PeakDetector> detectors_;
  std::vector<RowDetections> selected_;
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
