cmake_minimum_required(VERSION 3.5)
project(processing_lidar_objects)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

# Include AMENT build tool
find_package(ament_cmake REQUIRED)

# Now include other ROS2 packages
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

#
# Build Libs
#
add_library(scan_merger src/scans_merger.cpp)
ament_target_dependencies(scan_merger
  rclcpp
  std_srvs
  laser_geometry
  sensor_msgs
  geometry_msgs
)
