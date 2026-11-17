cmake_minimum_required(VERSION 3.10)
project(nav_ross)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Direct IDL preserves the legacy lowercase type and mixed-case field names.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HighPrecisionFFTData.idl"
  "msg/nav_msg.idl"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
get_filename_component(interface_source_dir "${CMAKE_CURRENT_SOURCE_DIR}" REALPATH)
install(FILES "${interface_source_dir}/../LICENSE" "${interface_source_dir}/../THIRD_PARTY.md"
  DESTINATION share/${PROJECT_NAME})

ament_package()
