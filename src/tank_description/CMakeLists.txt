cmake_minimum_required(VERSION 3.5)
project(tank_description)

find_package(ament_cmake REQUIRED)

# Install package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install config and launch
install(
  DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
