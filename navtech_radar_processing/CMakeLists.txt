cmake_minimum_required(VERSION 3.10)
project(navtech_radar_processing)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(navtech_radar_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(OpenMP REQUIRED COMPONENTS CXX)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(fft_sweep_assembler_component SHARED src/fft_sweep_assembler.cpp)
target_compile_features(fft_sweep_assembler_component PRIVATE cxx_std_17)
target_link_libraries(fft_sweep_assembler_component PRIVATE
  OpenMP::OpenMP_CXX
  rclcpp_components::component
  tf2_geometry_msgs::tf2_geometry_msgs tf2_ros::tf2_ros
  rclcpp::rclcpp
  navtech_radar_msgs::navtech_radar_msgs__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  diagnostic_msgs::diagnostic_msgs__rosidl_typesupport_cpp
  rcl_interfaces::rcl_interfaces__rosidl_typesupport_cpp
)
rclcpp_components_register_node(fft_sweep_assembler_component
  PLUGIN "radar::SweepAssembler" EXECUTABLE fft_sweep_assembler)
add_library(fft_sweep_visualizer_component SHARED src/fft_sweep_visualizer.cpp src/sweep_renderer.cpp)
target_compile_features(fft_sweep_visualizer_component PRIVATE cxx_std_17)
target_link_libraries(fft_sweep_visualizer_component PRIVATE
  rclcpp_components::component
  rclcpp::rclcpp
  navtech_radar_msgs::navtech_radar_msgs__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  rcl_interfaces::rcl_interfaces__rosidl_typesupport_cpp
  tf2::tf2 tf2_ros::tf2_ros opencv_core opencv_imgproc
)
rclcpp_components_register_node(fft_sweep_visualizer_component
  PLUGIN "radar::SweepVisualizer" EXECUTABLE fft_sweep_visualizer)
install(TARGETS fft_sweep_assembler_component fft_sweep_visualizer_component DESTINATION lib)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_ros_isolated_pytest_test(test_fft_sweep_assembler test
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test/reference:${CMAKE_CURRENT_SOURCE_DIR}/../navtech_radar_driver/test/reference:${CMAKE_CURRENT_SOURCE_DIR}/../tools
    TIMEOUT 90
  )
endif()

install(FILES ../LICENSE ../THIRD_PARTY.md DESTINATION share/${PROJECT_NAME})

ament_package()
