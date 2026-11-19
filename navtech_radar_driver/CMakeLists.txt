cmake_minimum_required(VERSION 3.10)
project(navtech_radar_driver)

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)
find_package(ZLIB REQUIRED)
find_package(BZip2 REQUIRED)
find_package(LibArchive REQUIRED)
find_package(libzip REQUIRED)
find_package(nlohmann_json REQUIRED)
find_library(LZ4_LIBRARY NAMES lz4 REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_ross REQUIRED)
find_package(navtech_radar_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

add_library(legacy_bag_driver_component SHARED src/legacy_bag_driver.cpp)
target_compile_features(legacy_bag_driver_component PRIVATE cxx_std_17)
target_link_libraries(legacy_bag_driver_component PRIVATE
  rclcpp_components::component
  rclcpp::rclcpp
  nav_ross::nav_ross__rosidl_typesupport_cpp
  navtech_radar_msgs::navtech_radar_msgs__rosidl_typesupport_cpp
  diagnostic_msgs::diagnostic_msgs__rosidl_typesupport_cpp
  rcl_interfaces::rcl_interfaces__rosidl_typesupport_cpp
)
rclcpp_components_register_node(legacy_bag_driver_component
  PLUGIN "LegacyBagDriver" EXECUTABLE legacy_bag_driver)
install(TARGETS legacy_bag_driver_component DESTINATION lib)

add_executable(recording_player src/recording_player.cpp src/dataset_files.cpp
  src/dataset_reader.cpp src/colraw_reader.cpp src/ros1_reader.cpp)
target_compile_features(recording_player PRIVATE cxx_std_17)
target_link_libraries(recording_player PRIVATE rclcpp::rclcpp
  navtech_radar_msgs::navtech_radar_msgs__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  rosgraph_msgs::rosgraph_msgs__rosidl_typesupport_cpp
  opencv_core opencv_imgcodecs ZLIB::ZLIB BZip2::BZip2 LibArchive::LibArchive
  libzip::zip nlohmann_json::nlohmann_json ${LZ4_LIBRARY})
add_executable(road_cloud_adapter src/road_cloud_adapter.cpp)
target_link_libraries(road_cloud_adapter PRIVATE rclcpp::rclcpp sensor_msgs::sensor_msgs__rosidl_typesupport_cpp)
add_executable(bag_player src/bag_player.cpp)
target_link_libraries(bag_player PRIVATE rosbag2_transport::rosbag2_transport)
install(TARGETS recording_player road_cloud_adapter bag_player DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_ros_isolated_pytest_test(test_legacy_bag_driver test
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test/reference
    TIMEOUT 90
  )
endif()

install(FILES ../LICENSE ../THIRD_PARTY.md DESTINATION share/${PROJECT_NAME})

ament_package()
