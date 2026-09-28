cmake_minimum_required(VERSION 3.10)
project(navtech_radar_driver)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/legacy_bag_driver.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME legacy_bag_driver
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_ros_isolated_pytest_test(test_legacy_bag_driver test
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
    TIMEOUT 90
  )
endif()

ament_package()
