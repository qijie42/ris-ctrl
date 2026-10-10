cmake_minimum_required(VERSION 3.5)
project(airsim_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# 安装 Python 脚本为可执行文件
install(PROGRAMS
  src/transform_node.py
  src/getLidarData.py
  src/move.py
  src/airsim_pose.py
  src/getCameraData.py
  DESTINATION lib/ 
)

# 安装 launch 文件
install(DIRECTORY launch
  DESTINATION share/ 
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
