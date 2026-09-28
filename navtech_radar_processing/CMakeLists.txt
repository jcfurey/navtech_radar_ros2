cmake_minimum_required(VERSION 3.10)
project(navtech_radar_processing)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/fft_sweep_assembler.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME fft_sweep_assembler
)

ament_package()
