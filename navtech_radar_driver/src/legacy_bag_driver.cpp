// SPDX-License-Identifier: Apache-2.0
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "nav_ross/msg/high_precision_fft_data.hpp"
#include "nav_ross/msg/nav_msg.hpp"
#include "navtech_radar_msgs/msg/fft_spoke.hpp"
#include "navtech_radar_msgs/msg/radar_configuration.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using Config = navtech_radar_msgs::msg::RadarConfiguration;
using Spoke = navtech_radar_msgs::msg::FftSpoke;
using LegacyConfig = nav_ross::msg::nav_msg;
using LegacyFFT = nav_ross::msg::HighPrecisionFFTData;
using Diagnostics = diagnostic_msgs::msg::DiagnosticArray;
using Steady = std::chrono::steady_clock;

class LegacyBagDriver final : public rclcpp::Node {
public:
  explicit LegacyBagDriver(const rclcpp::NodeOptions &options)
      : Node("navtech_legacy_bag_driver", options) {
    const auto input_config = param<std::string>(
        "legacy_configuration_topic", "/Navtech/Configuration_Topic");
    const auto input_fft =
        param<std::string>("legacy_fft_topic", "/Navtech/FFTData");
    frame_ = param<std::string>("frame_id", "navtech");
    const auto in_depth = param<int>("input_queue_depth", 512);
    const auto out_depth = param<int>("output_queue_depth", 512);
    if (frame_.empty() || frame_.front() == '/' || in_depth < 1 ||
        out_depth < 1)
      throw std::invalid_argument("invalid frame_id or queue depth");

    configs_ = create_publisher<Config>(
        param<std::string>("configuration_topic",
                           "navtech_radar/configuration"),
        rclcpp::QoS(1).transient_local());
    spokes_ = create_publisher<Spoke>(
        param<std::string>("spoke_topic", "navtech_radar/fft/spoke"),
        rclcpp::QoS(out_depth).best_effort());
    diagnostics_ = create_publisher<Diagnostics>("diagnostics", 10);
    const auto qos = rclcpp::QoS(in_depth).best_effort();
    config_sub_ = create_subscription<LegacyConfig>(
        input_config, qos, [this](const LegacyConfig &m) { configure(m); });
    fft_sub_ = create_subscription<LegacyFFT>(
        input_fft, qos, [this](LegacyFFT::UniquePtr m) { adapt(*m); });
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { report(); });
  }

private:
  template <class T> T param(const std::string &name, T value) {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.read_only = true;
    return declare_parameter<T>(name, value, desc);
  }

  void configure(const LegacyConfig &m) {
    ++config_count_;
    if (!std::isfinite(m.range_resolution) || m.range_resolution <= 0 ||
        !m.AzimuthSamples || m.AzimuthSamples > m.EncoderSize || !m.BinSize ||
        !m.RangeInBins || !m.ExpectedRotationRate ||
        uint32_t(m.AzimuthSamples) * m.ExpectedRotationRate > UINT16_MAX) {
      ++invalid_configs_;
      return;
    }
    const auto key =
        std::make_tuple(m.range_resolution, m.AzimuthSamples, m.EncoderSize,
                        m.BinSize, m.RangeInBins, m.ExpectedRotationRate);
    if (key_ == key)
      return;
    key_ = key;
    Config c;
    c.header.stamp = now();
    c.header.frame_id = frame_;
    c.configuration_version = ++version_;
    c.range_resolution = m.range_resolution;
    c.azimuth_samples = m.AzimuthSamples;
    c.encoder_size = m.EncoderSize;
    c.bin_size = m.BinSize;
    c.range_bins = m.RangeInBins;
    c.expected_rotation_rate = m.ExpectedRotationRate;
    c.packet_rate = m.AzimuthSamples * m.ExpectedRotationRate;
    c.range_gain = 1;
    c.sample_bit_depth = 16;
    bins_ = c.range_bins;
    encoder_size_ = c.encoder_size;
    configs_->publish(c);
    RCLCPP_INFO(get_logger(),
                "Configuration %u: %u spokes, %u bins, %.6f m/bin", version_,
                unsigned(c.azimuth_samples), unsigned(c.range_bins),
                c.range_resolution);
  }

  void adapt(LegacyFFT &m) {
    if (!std::isfinite(m.angle) || m.data.empty() ||
        m.data.size() > UINT16_MAX || m.size != m.data.size() ||
        m.header.stamp.sec < 0 || m.header.stamp.nanosec >= 1000000000u ||
        (key_ && (m.size != bins_ || m.azimuth >= encoder_size_))) {
      ++invalid_spokes_;
      return;
    }
    auto out = std::make_unique<Spoke>();
    out->header.stamp = m.header.stamp;
    out->header.frame_id = frame_;
    out->host_stamp = now();
    out->configuration_version = version_;
    out->ntp_seconds = m.header.stamp.sec;
    out->ntp_split_seconds = m.header.stamp.nanosec;
    out->sweep_counter = m.sweepCounter;
    out->azimuth = m.azimuth;
    auto degrees = std::fmod(double(m.angle), 360.0);
    out->azimuth_angle =
        (degrees < 0 ? degrees + 360 : degrees) * (std::acos(-1.0) / 180.0);
    out->sample_bit_depth = 16;
    out->data = std::move(m.data);
    spokes_->publish(std::move(out));
    ++spoke_count_;
    last_spoke_ = Steady::now();
  }

  void report() {
    Diagnostics report;
    report.header.stamp = now();
    auto &status = report.status.emplace_back();
    const auto stamp = Steady::now();
    const bool recent =
        spoke_count_ && stamp - last_spoke_ < std::chrono::seconds(2);
    status.name =
        std::string(get_fully_qualified_name()) + ": legacy playback transport";
    status.hardware_id = "navtech-bag-playback";
    status.level = recent ? status.OK : status.WARN;
    status.message = recent ? "Receiving high-precision FFT spokes"
                            : "Waiting for high-precision FFT spokes";
    auto add = [&](const char *key, auto value) {
      auto &kv = status.values.emplace_back();
      kv.key = key;
      kv.value = std::to_string(value);
    };
    add("configuration_version", version_);
    add("configuration_messages", config_count_);
    add("invalid_configurations", invalid_configs_);
    add("spokes", spoke_count_);
    add("spoke_rate_hz",
        (spoke_count_ - last_count_) /
            std::chrono::duration<double>(stamp - last_report_).count());
    add("invalid_spokes", invalid_spokes_);
    add("sample_bit_depth", 16);
    last_count_ = spoke_count_;
    last_report_ = stamp;
    diagnostics_->publish(report);
  }

  std::string frame_;
  std::optional<
      std::tuple<double, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t>>
      key_;
  uint16_t bins_{}, encoder_size_{};
  uint32_t version_{};
  uint64_t config_count_{}, invalid_configs_{}, spoke_count_{},
      invalid_spokes_{}, last_count_{};
  Steady::time_point last_report_{Steady::now()}, last_spoke_{};
  rclcpp::Publisher<Config>::SharedPtr configs_;
  rclcpp::Publisher<Spoke>::SharedPtr spokes_;
  rclcpp::Publisher<Diagnostics>::SharedPtr diagnostics_;
  rclcpp::Subscription<LegacyConfig>::SharedPtr config_sub_;
  rclcpp::Subscription<LegacyFFT>::SharedPtr fft_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(LegacyBagDriver)
