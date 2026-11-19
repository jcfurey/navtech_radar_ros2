// SPDX-License-Identifier: Apache-2.0
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <array>
#include <cstring>

using Cloud = sensor_msgs::msg::PointCloud2;
using Clock = std::chrono::steady_clock;
void normalize(Cloud &m, const std::string &frame) {
  static const std::array<std::string, 4> names = {"x", "y", "z", "intensity"};
  if (m.fields.size() != 4 || m.height != 1 || m.point_step != 16 ||
      uint64_t(m.width) * 16 != m.row_step || m.data.size() != m.row_step)
    throw std::invalid_argument("unexpected Navtech road cloud layout");
  for (size_t i = 0; i < 4; ++i) {
    auto &f = m.fields[i];
    if (f.name != names[i] || f.offset != 4 * i || f.datatype != f.FLOAT32 ||
        f.count != m.width)
      throw std::invalid_argument("unexpected Navtech road cloud fields");
    f.count = 1;
  }
  uint16_t one = 1;
  bool swap = m.is_bigendian != (*reinterpret_cast<uint8_t *>(&one) == 0);
  for (size_t i = 0; i < m.data.size(); i += 16) {
    for (size_t axis = 0; axis < 2; ++axis) {
      auto p = m.data.data() + i + axis * 4;
      uint32_t bits;
      std::memcpy(&bits, p, 4);
      if (swap)
        bits = __builtin_bswap32(bits);
      float v;
      std::memcpy(&v, &bits, 4);
      v *= axis == 0 ? 0.0438f : -0.0438f;
      std::memcpy(&bits, &v, 4);
      if (swap)
        bits = __builtin_bswap32(bits);
      std::memcpy(p, &bits, 4);
    }
  }
  m.header.frame_id = frame;
}
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<rclcpp::Node>("navtech_road_cloud_adapter");
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.read_only = true;
    auto frame =
        node->declare_parameter<std::string>("frame_id", "navtech", desc);
    if (frame.empty() || frame.front() == '/')
      throw std::invalid_argument("invalid frame_id");
    auto out = node->create_publisher<Cloud>("navtech_radar/point_cloud", 10);
    Clock::time_point last{};
    auto input = node->create_subscription<Cloud>("~/input", 10,
                                                  [&](Cloud::UniquePtr m) {
                                                    normalize(*m, frame);
                                                    out->publish(std::move(m));
                                                    last = Clock::now();
                                                  });
    auto timer = node->create_wall_timer(std::chrono::milliseconds(250), [&] {
      if (last != Clock::time_point{} &&
          Clock::now() - last >= std::chrono::milliseconds(250) &&
          !input->get_publisher_count()) {
        out->wait_for_all_acked(std::chrono::seconds(2));
        rclcpp::shutdown();
      }
    });
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    if (rclcpp::ok())
      rclcpp::shutdown();
    return 1;
  }
  if (rclcpp::ok())
    rclcpp::shutdown();
  return 0;
}
