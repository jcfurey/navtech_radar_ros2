cmake_minimum_required(VERSION 3.10)
project(navtech_radar_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FftSpoke.msg"
  "msg/FftSweep.msg"
  "msg/RadarConfiguration.msg"
  DEPENDENCIES builtin_interfaces sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
install(FILES ../LICENSE ../THIRD_PARTY.md DESTINATION share/${PROJECT_NAME})

ament_package()
