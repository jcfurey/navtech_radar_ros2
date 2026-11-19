// SPDX-License-Identifier: Apache-2.0
#include "rclcpp/rclcpp.hpp"
#include "recording.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include <charconv>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <map>
#include <thread>

namespace recording {
using Clock = std::chrono::steady_clock;
struct Options {
  fs::path path;
  std::string profile = "auto", frame = "navtech";
  double rate = 1, gap = 0, delay = 2;
  size_t limit = 0;
  bool clock = false, wait = false;
};
Options options(const std::vector<std::string> &args) {
  require(args.size() >= 2,
          "usage: recording_player RECORDING [--format PROFILE] [--rate RATE]");
  Options o;
  auto path = args[1];
  if (path.rfind("~/", 0) == 0 && std::getenv("HOME"))
    o.path = fs::path(std::getenv("HOME")) / path.substr(2);
  else
    o.path = path;
  for (size_t i = 2; i < args.size(); ++i) {
    const auto &key = args[i];
    if (key == "--clock") {
      o.clock = true;
      continue;
    }
    if (key == "--wait-for-assembler") {
      o.wait = true;
      continue;
    }
    require(++i < args.size(), "missing value for " + key);
    const auto &v = args[i];
    if (key == "--format")
      o.profile = v;
    else if (key == "--frame-id")
      o.frame = v;
    else if (key == "--limit") {
      auto res = std::from_chars(v.data(), v.data() + v.size(), o.limit);
      require(res.ec == std::errc{} && res.ptr == v.data() + v.size(),
              "invalid frame limit");
    } else {
      size_t end;
      double n = std::stod(v, &end);
      require(end == v.size() && std::isfinite(n) && n >= 0,
              "invalid value for " + key);
      if (key == "--rate")
        o.rate = n;
      else if (key == "--max-gap")
        o.gap = n;
      else if (key == "--delay")
        o.delay = n;
      else
        throw std::runtime_error("unknown option " + key);
    }
  }
  require(o.rate > 0 && !o.frame.empty() && o.frame.front() != '/',
          "invalid playback rate or frame");
  if (o.profile == "auto")
    o.profile = identify(o.path);
  return o;
}
struct Finished {};
void check_running() {
  if (!rclcpp::ok())
    throw Finished{};
}
void play(const Options &o) {
  auto node = std::make_shared<rclcpp::Node>("navtech_recording_player");
  auto clock =
      o.clock ? node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10)
              : nullptr;
  using Pub = rclcpp::PublisherBase::SharedPtr;
  std::map<std::string, Pub> pubs;
  // Advertise products before the initial discovery delay.
  if (o.profile == "ros1" || o.profile == "radiate_cartesian") {
    pubs["fft/cartesian"] =
        node->create_publisher<Image>("navtech_radar/fft/cartesian", 10);
    if (o.profile == "ros1") {
      pubs["fft/display"] =
          node->create_publisher<Image>("navtech_radar/fft/display", 10);
      pubs["scan"] = node->create_publisher<Scan>("navtech_radar/scan", 10);
      pubs["point_cloud"] =
          node->create_publisher<Cloud>("navtech_radar/point_cloud", 10);
    }
  } else {
    auto rotation =
        node->create_publisher<Rotation>("navtech_radar/recording/rotation", 4);
    if (o.wait) {
      auto end = Clock::now() + std::chrono::seconds(30);
      while (rclcpp::ok() && !rotation->get_subscription_count()) {
        require(Clock::now() < end,
                "assembler did not subscribe to recorded rotations");
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }
    pubs["recording/rotation"] = rotation;
  }
  size_t count = 0;
  uint64_t previous = 0;
  Clock::time_point due;
  navtech_radar_msgs::msg::RadarConfiguration last;
  uint32_t version = 0;
  Emit emit = [&](uint64_t ns, const char *topic, Product product) {
    double seconds = o.delay;
    if (count) {
      seconds = ns > previous ? (ns - previous) * 1e-9 : 0;
      if (o.gap)
        seconds = std::min(seconds, o.gap);
      seconds /= o.rate;
    } else
      due = Clock::now();
    due += std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(seconds));
    while (rclcpp::ok() && Clock::now() < due)
      std::this_thread::sleep_until(
          std::min(due, Clock::now() + std::chrono::milliseconds(50)));
    check_running();
    if (clock) {
      rosgraph_msgs::msg::Clock m;
      m.clock = stamp(ns);
      clock->publish(m);
    }
    std::visit(
        [&](auto &m) {
          using M = std::decay_t<decltype(m)>;
          if constexpr (std::is_same_v<M, Rotation>) {
            auto &c = m.configuration;
            if (!version || c != last) {
              last = c;
              ++version;
            }
            m.image.header.frame_id = o.frame;
            m.image.header.stamp = stamp(ns);
            c.header = m.image.header;
            c.configuration_version = version;
            m.source_format = o.profile;
          } else if constexpr (std::is_same_v<M, Image>) {
            if (o.profile != "ros1") {
              m.header.frame_id = o.frame;
              m.header.stamp = stamp(ns);
            }
          }
          auto pub =
              std::static_pointer_cast<rclcpp::Publisher<M>>(pubs.at(topic));
          pub->publish(std::make_unique<M>(std::move(m)));
        },
        product);
    previous = ns;
    if (++count == o.limit)
      throw Finished{};
  };
  try {
    if (o.profile == "ros1")
      read_ros1(o.path, emit);
    else if (o.profile == "colraw" || o.profile == "cpr")
      read_colraw(o.path, o.profile == "cpr", emit);
    else
      read_dataset(o.path, o.profile, emit);
  } catch (const Finished &) {
  }
  if (rclcpp::ok()) {
    require(count > 0, "recording contained no radar measurements");
    for (auto &[name, pub] : pubs)
      pub->wait_for_all_acked(std::chrono::seconds(2));
    RCLCPP_INFO(node->get_logger(), "Finished %zu recorded frames/messages",
                count);
  }
}
} // namespace recording
int main(int argc, char **argv) {
  try {
    if (argc == 2 && (std::string_view(argv[1]) == "--help" ||
                      std::string_view(argv[1]) == "-h")) {
      std::cout << "Usage: recording_player RECORDING [--format PROFILE] "
                   "[--rate RATE] [--frame-id FRAME]\n"
                   "       [--delay SECONDS] [--max-gap SECONDS] [--limit "
                   "COUNT] [--clock] [--wait-for-assembler]\n"
                   "       recording_player --identify RECORDING\n";
      return 0;
    }
    if (argc == 3 && std::string(argv[1]) == "--identify") {
      std::cout << recording::identify(argv[2]) << '\n';
      return 0;
    }
    auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
    recording::play(recording::options(args));
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "recording_player: " << e.what() << '\n';
    if (rclcpp::ok())
      rclcpp::shutdown();
    return 1;
  }
}
