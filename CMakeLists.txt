cmake_minimum_required(VERSION 3.8)
project(ursula)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install Python scripts (each must be executable: chmod +x scripts/*.py)
install(PROGRAMS
  scripts/serial_command_bridge.py
  scripts/ursula_manager.py
  scripts/detection_mapper.py
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

# Install models directory if it exists (YOLO blobs)
# Create with: mkdir -p models && wget <blob-url> -P models/
install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL   # Does not fail build if models/ doesn't exist yet
)

ament_package()