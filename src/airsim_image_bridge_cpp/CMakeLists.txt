cmake_minimum_required(VERSION 3.8)
project(airsim_image_bridge_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-O3 -Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

set(AIRSIM_ROOT "$ENV{HOME}/AirSim")

find_path(AIRSIM_INCLUDE_DIR
  NAMES vehicles/multirotor/api/MultirotorRpcLibClient.hpp
  PATHS "${AIRSIM_ROOT}/AirLib/include"
  NO_DEFAULT_PATH
)

find_library(AIRSIM_AIRLIB_LIBRARY
  NAMES AirLib
  PATHS "${AIRSIM_ROOT}/AirLib/lib"
  NO_DEFAULT_PATH
)

find_library(AIRSIM_RPC_LIBRARY
  NAMES rpc
  PATHS "${AIRSIM_ROOT}/AirLib/deps/rpclib/lib"
  NO_DEFAULT_PATH
)

if(NOT AIRSIM_INCLUDE_DIR)
  message(FATAL_ERROR "AirSim C++ headers not found in ${AIRSIM_ROOT}")
endif()

if(NOT AIRSIM_AIRLIB_LIBRARY)
  message(FATAL_ERROR "AirLib library not found in ${AIRSIM_ROOT}/AirLib/lib")
endif()

if(NOT AIRSIM_RPC_LIBRARY)
  message(FATAL_ERROR "rpclib library not found in ${AIRSIM_ROOT}/AirLib/deps/rpclib/lib")
endif()

add_executable(airsim_image_bridge
  src/airsim_image_bridge.cpp
)

target_include_directories(airsim_image_bridge PRIVATE
  "${AIRSIM_INCLUDE_DIR}"
  "${AIRSIM_ROOT}/AirLib/deps/rpclib/include"
  "${AIRSIM_ROOT}/AirLib/deps/eigen3"
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(airsim_image_bridge
  rclcpp
  sensor_msgs
)

target_link_libraries(airsim_image_bridge
  "${AIRSIM_AIRLIB_LIBRARY}"
  "${AIRSIM_RPC_LIBRARY}"
  Eigen3::Eigen
  pthread
)

install(TARGETS airsim_image_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
