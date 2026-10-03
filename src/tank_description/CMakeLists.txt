cmake_minimum_required(VERSION 3.18)
project(tank_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_program(XACRO_EXECUTABLE xacro REQUIRED)

set(ROBOT_XACRO "${CMAKE_CURRENT_SOURCE_DIR}/urdf/robot.urdf.xacro")
set(ROBOT_URDF "${CMAKE_CURRENT_BINARY_DIR}/urdf/robot.urdf")
add_custom_command(
  OUTPUT "${ROBOT_URDF}"
  COMMAND "${CMAKE_COMMAND}" -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/urdf"
  COMMAND "${XACRO_EXECUTABLE}" "${ROBOT_XACRO}" -o "${ROBOT_URDF}"
  DEPENDS "${ROBOT_XACRO}"
  VERBATIM
)
add_custom_target(robot_description ALL DEPENDS "${ROBOT_URDF}")

install(FILES "${ROBOT_XACRO}" "${ROBOT_URDF}" DESTINATION share/${PROJECT_NAME}/urdf)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_robot_description test/test_robot_description.py
    ENV "ROBOT_URDF_PATH=${ROBOT_URDF}"
  )
endif()

ament_package()
