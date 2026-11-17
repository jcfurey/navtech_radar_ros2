cmake_minimum_required(VERSION 3.10)
project(navtech_radar_driver)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(nav_ross REQUIRED)
find_package(navtech_radar_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

add_executable(legacy_bag_driver src/legacy_bag_driver.cpp)
target_compile_features(legacy_bag_driver PRIVATE cxx_std_17)
target_link_libraries(legacy_bag_driver PRIVATE
  rclcpp::rclcpp
  nav_ross::nav_ross__rosidl_typesupport_cpp
  navtech_radar_msgs::navtech_radar_msgs__rosidl_typesupport_cpp
  diagnostic_msgs::diagnostic_msgs__rosidl_typesupport_cpp
  rcl_interfaces::rcl_interfaces__rosidl_typesupport_cpp
)
install(TARGETS legacy_bag_driver DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/legacy_bag_driver.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME legacy_bag_driver_py
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_ros_isolated_pytest_test(test_legacy_bag_driver test
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
    TIMEOUT 90
  )
endif()

install(FILES ../LICENSE ../THIRD_PARTY.md DESTINATION share/${PROJECT_NAME})

ament_package()
