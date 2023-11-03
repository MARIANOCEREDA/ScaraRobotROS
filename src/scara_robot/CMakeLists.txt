cmake_minimum_required(VERSION 3.8)
project(scara_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include/scara_robot)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(trajectory_msgs REQUIRED)

set(ROS_LIBS
  rclcpp
  example_interfaces
  trajectory_msgs
)

set(HEADER_FILES
  include/scara_robot/inverse_kinematics_node.hpp
  include/scara_robot/robot_dimensions.hpp
)

add_executable(inverse_kinematics_node src/inverse_kinematics_node.cpp ${HEADER_FILES})
ament_target_dependencies(inverse_kinematics_node ${ROS_LIBS})

install(
  TARGETS
  inverse_kinematics_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
