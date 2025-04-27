cmake_minimum_required(VERSION 3.5)
project(joystick)

# Find ament_cmake and dependencies
find_package(ament_cmake REQUIRED)

# Install launch files and configurations
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
