// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "navtech_radar_msgs/msg/recorded_rotation.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <cstdint>
#include <filesystem>
#include <functional>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

namespace recording {
namespace fs = std::filesystem;
using Rotation = navtech_radar_msgs::msg::RecordedRotation;
using Image = sensor_msgs::msg::Image;
using Cloud = sensor_msgs::msg::PointCloud2;
using Scan = sensor_msgs::msg::LaserScan;
using Bytes = std::vector<uint8_t>;
using Product = std::variant<Rotation, Image, Cloud, Scan>;
using Emit = std::function<void(uint64_t, const char *, Product)>;

inline uint64_t little(const uint8_t *p, unsigned bytes) {
  uint64_t n = 0;
  for (unsigned i = 0; i < bytes; ++i)
    n |= uint64_t(p[i]) << (8 * i);
  return n;
}
inline uint32_t big(const uint8_t *p, unsigned bytes) {
  uint32_t n = 0;
  for (unsigned i = 0; i < bytes; ++i)
    n = (n << 8) | p[i];
  return n;
}
inline void require(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}
inline void require(bool ok, const std::string &message) {
  if (!ok)
    throw std::runtime_error(message);
}
inline builtin_interfaces::msg::Time stamp(uint64_t ns) {
  require(ns / 1000000000 <= INT32_MAX, "timestamp outside ROS Time range");
  builtin_interfaces::msg::Time t;
  t.sec = ns / 1000000000;
  t.nanosec = ns % 1000000000;
  return t;
}

class TemporaryDirectory {
public:
  TemporaryDirectory();
  ~TemporaryDirectory();
  TemporaryDirectory(const TemporaryDirectory &) = delete;
  TemporaryDirectory &operator=(const TemporaryDirectory &) = delete;
  fs::path path;
};

void check_running();
std::string identify(const fs::path &path);
void read_dataset(const fs::path &, const std::string &profile, const Emit &);
void read_colraw(const fs::path &, bool cpr, const Emit &);
void read_ros1(const fs::path &, const Emit &);
} // namespace recording
