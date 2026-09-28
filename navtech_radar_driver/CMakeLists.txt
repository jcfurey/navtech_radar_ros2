cmake_minimum_required(VERSION 3.10)
project(navtech_radar_driver)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/legacy_bag_driver.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME legacy_bag_driver
)

ament_package()
