cmake_minimum_required(VERSION 3.10)
project(navtech_radar_processing)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(navtech_radar_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(fft_sweep_assembler src/fft_sweep_assembler.cpp)
target_compile_features(fft_sweep_assembler PRIVATE cxx_std_17)
target_link_libraries(fft_sweep_assembler PRIVATE
  rclcpp::rclcpp
  navtech_radar_msgs::navtech_radar_msgs__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  rcl_interfaces::rcl_interfaces__rosidl_typesupport_cpp
)
add_executable(fft_sweep_visualizer src/fft_sweep_visualizer.cpp src/sweep_renderer.cpp)
target_compile_features(fft_sweep_visualizer PRIVATE cxx_std_17)
target_link_libraries(fft_sweep_visualizer PRIVATE
  rclcpp::rclcpp
  navtech_radar_msgs::navtech_radar_msgs__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  rcl_interfaces::rcl_interfaces__rosidl_typesupport_cpp
  tf2::tf2 tf2_ros::tf2_ros opencv_core opencv_imgproc
)
install(TARGETS fft_sweep_assembler fft_sweep_visualizer DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/fft_sweep_assembler.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fft_sweep_assembler_py
)

install(PROGRAMS
  scripts/fft_sweep_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fft_sweep_visualizer_py
)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_ros_isolated_pytest_test(test_fft_sweep_assembler test
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts:${CMAKE_CURRENT_SOURCE_DIR}/../tools
    TIMEOUT 90
  )
endif()

install(FILES ../LICENSE ../THIRD_PARTY.md DESTINATION share/${PROJECT_NAME})

ament_package()
