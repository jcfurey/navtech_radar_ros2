// SPDX-License-Identifier: Apache-2.0
#include "sweep_renderer.hpp"
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace radar {
class SweepVisualizer final : public rclcpp::Node {
public:
  SweepVisualizer()
      : Node("navtech_fft_sweep_visualizer"),
        renderer_(
            {param("min_range", 2.0), param("max_range", 150.0),
             param<int64_t>("image_width", 1000),
             param<int64_t>("cartesian_size", 800),
             param("lower_percentile", 50.0), param("upper_percentile", 99.7),
             param("contrast_alpha", 0.2), param("gamma", 1.0),
             param("annotate", true), param("azimuth_offset_degrees", 0.0)}),
        robot_(param<std::string>("robot_frame", "")),
        heading_(param<std::string>("heading_frame", "")),
        buffer_(get_clock(), tf2::BUFFER_CORE_DEFAULT_CACHE_TIME,
                tf2_ros::Buffer::RequiredInterfaces(*this)),
        listener_(buffer_,
                  tf2_ros::TransformListener::RequiredInterfaces(*this),
                  false) {
    if (robot_.find('/') == 0 || heading_.find('/') == 0)
      throw std::invalid_argument("TF frame names must have no leading slash");
    polar_ = create_publisher<Image>(
        param<std::string>("polar_topic", "navtech_radar/fft/display"), 1);
    cartesian_ = create_publisher<Image>(
        param<std::string>("cartesian_topic", "navtech_radar/fft/cartesian"),
        1);
    configs_ = create_subscription<Config>(
        param<std::string>("configuration_topic",
                           "navtech_radar/configuration"),
        rclcpp::QoS(1).transient_local(), [this](Config::ConstSharedPtr c) {
          if (SweepRenderer::valid_configuration(*c))
            config_ = std::move(c);
          else
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Ignoring invalid display configuration");
        });
    sweeps_ = create_subscription<Sweep>(
        param<std::string>("sweep_topic", "navtech_radar/fft/sweep"), 1,
        [this](Sweep::ConstSharedPtr s) { display(*s); });
  }

private:
  template <class T> T param(const std::string &name, T value) {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.read_only = true;
    return declare_parameter<T>(name, value, desc);
  }

  std::optional<double> yaw(const std::string &target,
                            const builtin_interfaces::msg::Time &stamp) const {
    if (target.empty() || robot_.empty() ||
        (stamp.sec == 0 && stamp.nanosec == 0))
      return {};
    try {
      const tf2::TimePoint time{std::chrono::seconds(stamp.sec) +
                                std::chrono::nanoseconds(stamp.nanosec)};
      const auto transform = buffer_.lookupTransform(target, robot_, time);
      const auto &q = transform.transform.rotation;
      const double norm =
          std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
      if (!std::isfinite(norm) || norm < 1e-12)
        return {};
      const double x = q.x / norm, y = q.y / norm, z = q.z / norm,
                   w = q.w / norm;
      const double fx = 1 - 2 * (y * y + z * z), fy = 2 * (x * y + w * z);
      if (std::hypot(fx, fy) >= 1e-6)
        return std::atan2(fy, fx);
    } catch (const tf2::TransformException &) {
      // A missing transform is represented in the annotation for this sweep.
    }
    return {};
  }

  void display(const Sweep &s) {
    if (!config_)
      return;
    auto polar =
        polar_->get_subscription_count() ? std::make_unique<Image>() : nullptr;
    auto plan = cartesian_->get_subscription_count() ? std::make_unique<Image>()
                                                     : nullptr;
    if (!polar && !plan)
      return;
    try {
      const DisplayHeading overlay{yaw(s.header.frame_id, s.header.stamp),
                                   yaw(heading_, s.header.stamp), heading_};
      renderer_.render(s, *config_, overlay, polar.get(), plan.get());
    } catch (const std::invalid_argument &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", e.what());
      return;
    }
    if (polar)
      polar_->publish(std::move(polar));
    if (plan)
      cartesian_->publish(std::move(plan));
  }

  SweepRenderer renderer_;
  std::string robot_, heading_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  Config::ConstSharedPtr config_;
  rclcpp::Publisher<Image>::SharedPtr polar_, cartesian_;
  rclcpp::Subscription<Config>::SharedPtr configs_;
  rclcpp::Subscription<Sweep>::SharedPtr sweeps_;
};
} // namespace radar

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<radar::SweepVisualizer>());
  rclcpp::shutdown();
}
