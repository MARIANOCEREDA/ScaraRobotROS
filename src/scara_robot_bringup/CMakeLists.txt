cmake_minimum_required(VERSION 3.8)
project(scara_robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(scara_robot_description REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(scara_robot REQUIRED)
find_package(gazebo_ros REQUIRED)

install(
  DIRECTORY
  rviz
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
