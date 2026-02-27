cmake_minimum_required(VERSION 3.8)
project(ursula)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/serial_command_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install all resource directories
install(DIRECTORY
  config
  launch
  urdf
  worlds
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
