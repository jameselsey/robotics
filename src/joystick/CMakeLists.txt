cmake_minimum_required(VERSION 3.5)
project(joystick)

find_package(ament_cmake REQUIRED)

# Install package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install config and launch
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
