cmake_minimum_required(VERSION 3.10)
project(navtech_radar_processing)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/fft_sweep_assembler.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fft_sweep_assembler
)

install(PROGRAMS
  scripts/fft_sweep_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fft_sweep_visualizer
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_ros_isolated_pytest_test(test_fft_sweep_assembler test
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
    TIMEOUT 90
  )
endif()

ament_package()
