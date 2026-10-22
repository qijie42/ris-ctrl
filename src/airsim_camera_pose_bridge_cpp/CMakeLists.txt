cmake_minimum_required(VERSION 3.8)
project(airsim_camera_pose_bridge_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(
  odom_to_camera_pose
  src/odom_to_camera_pose.cpp
)

ament_target_dependencies(
  odom_to_camera_pose
  rclcpp
  nav_msgs
  geometry_msgs
)

install(
  TARGETS odom_to_camera_pose
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
