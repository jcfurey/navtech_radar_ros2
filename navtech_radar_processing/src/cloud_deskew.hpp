// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "planar_products.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include <chrono>
#include <optional>
#include <string>
#include <vector>

namespace radar {

// Only point-bearing spokes need a transform. end is the exclusive point index.
struct SpokePoints {
  builtin_interfaces::msg::Time stamp;
  uint32_t end;
};

class CloudDeskew {
public:
  CloudDeskew(rclcpp::Node &node, std::string fixed,
              rclcpp::Publisher<Cloud>::SharedPtr publisher)
      : node_(node), fixed_(std::move(fixed)), publisher_(std::move(publisher)),
        buffer_(node.get_clock(), tf2::BUFFER_CORE_DEFAULT_CACHE_TIME,
                tf2_ros::Buffer::RequiredInterfaces(node)),
        listener_(buffer_, tf2_ros::TransformListener::RequiredInterfaces(node),
                  false),
        timer_(node.create_wall_timer(std::chrono::milliseconds(50),
                                      [this] { retry(); })) {}

  void clear() { pending_.reset(); }

  void submit(Cloud cloud, std::vector<SpokePoints> spans) {
    retry(true);
    if (cloud.header.frame_id == fixed_ ||
        nanoseconds(cloud.header.stamp) <= 0 ||
        std::any_of(spans.begin(), spans.end(),
                    [](const auto &s) { return nanoseconds(s.stamp) <= 0; })) {
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                           "Skipping deskewed cloud: need positive acquisition "
                           "times and a distinct fixed frame");
      return;
    }
    pending_.emplace(Pending{std::move(cloud), std::move(spans),
                             Clock::now() + std::chrono::seconds(1)});
    retry();
  }

private:
  using Clock = std::chrono::steady_clock;
  struct Pending {
    Cloud cloud;
    std::vector<SpokePoints> spans;
    Clock::time_point deadline;
  };

  void retry(bool superseded = false) {
    if (!pending_)
      return;
    auto &cloud = pending_->cloud;
    const auto &spans = pending_->spans;
    const auto reference = rclcpp::Time(cloud.header.stamp);
    const auto &frame = cloud.header.frame_id;
    std::vector<tf2::Transform> poses(spans.size());
    try {
      // Look up the tail first: it is usually the sample still waiting for TF.
      for (size_t i = spans.size(); i-- > 0;) {
        auto transform = buffer_.lookupTransform(
            frame, reference, frame, rclcpp::Time(spans[i].stamp), fixed_);
        tf2::fromMsg(transform.transform, poses[i]);
      }
    } catch (const tf2::TransformException &error) {
      if (superseded || Clock::now() >= pending_->deadline) {
        RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                             "Dropping cloud without deskew TF: %s",
                             error.what());
        clear();
      }
      return;
    }
    uint32_t point = 0;
    for (size_t i = 0; i < spans.size(); ++i) {
      for (; point < spans[i].end; ++point) {
        auto *data = cloud.data.data() + size_t(point) * cloud.point_step;
        std::array<float, 3> xyz;
        std::memcpy(xyz.data(), data, sizeof(xyz));
        const auto p = poses[i] * tf2::Vector3(xyz[0], xyz[1], xyz[2]);
        xyz = {float(p.x()), float(p.y()), float(p.z())};
        std::memcpy(data, xyz.data(), sizeof(xyz));
      }
    }
    publisher_->publish(cloud);
    clear();
  }

  rclcpp::Node &node_;
  std::string fixed_;
  rclcpp::Publisher<Cloud>::SharedPtr publisher_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  std::optional<Pending> pending_;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace radar
