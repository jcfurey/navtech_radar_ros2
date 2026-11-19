// SPDX-License-Identifier: Apache-2.0
#include "rosbag2_transport/player.hpp"
#include <cmath>
#include <iostream>

// Use rosbag2's native transport and release its publishers at EOF, including
// the upstream publisher watched by the road cloud adapter.
int main(int argc, char **argv) {
  try {
    auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
    if (args.size() < 2)
      throw std::invalid_argument("missing bag directory");
    rosbag2_storage::StorageOptions storage;
    storage.uri = args[1];
    rosbag2_transport::PlayOptions play;
    play.clock_publish_frequency = 100;
    play.delay = rclcpp::Duration(2, 0);
    for (size_t i = 2; i < args.size(); ++i) {
      if (args[i] == "--rate" && i + 1 < args.size()) {
        const auto &rate = args[++i];
        size_t end;
        play.rate = std::stof(rate, &end);
        if (end != rate.size() || !std::isfinite(play.rate) || play.rate <= 0)
          throw std::invalid_argument("invalid rate");
      } else if (args[i] == "--canonical" && i + 2 < args.size()) {
        play.topic_qos_profile_overrides.emplace(
            args[++i], rclcpp::QoS(32).transient_local());
        play.topic_qos_profile_overrides.emplace(args[++i],
                                                 rclcpp::QoS(512).reliable());
      } else if (args[i] == "--wait-for-all-acked")
        play.wait_acked_timeout = 2000;
      else if (args[i] == "--topics") {
        play.topics_to_filter.assign(args.begin() + i + 1, args.end());
        break;
      } else
        throw std::invalid_argument("unknown bag playback option " + args[i]);
    }
    auto player = std::make_shared<rosbag2_transport::Player>(storage, play);
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(player);
    if (!player->play())
      throw std::runtime_error("cannot start bag playback");
    while (rclcpp::ok() &&
           !player->wait_for_playback_to_finish(std::chrono::milliseconds(10)))
      exec.spin_some();
    player->stop();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "bag_player: " << e.what() << '\n';
    if (rclcpp::ok())
      rclcpp::shutdown();
    return 1;
  }
}
