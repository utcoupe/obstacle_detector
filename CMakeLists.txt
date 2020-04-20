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
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_msgs REQUIRED)
# find_package(tf2_sensor_msgs REQUIRED)

# Generate msgs
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/CircleObstacle.msg
  msg/Obstacles.msg
  msg/SegmentObstacle.msg
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
  ADD_LINTER_TESTS
)

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
#  tf2
#  tf2_msgs
#  tf2_sensor_msgs
)

add_library(obstacle_extractor src/obstacle_extractor.cpp)
ament_target_dependencies(obstacle_extractor
  rclcpp
  std_srvs
  sensor_msgs
  geometry_msgs
)

add_library(obstacle_tracker src/obstacle_tracker.cpp)
ament_target_dependencies(obstacle_tracker
  rclcpp
  std_srvs
  sensor_msgs
  geometry_msgs
)

# Hacky way to depend on ROS msg/srv from the same package (as of Eloquent)
get_default_rmw_implementation(rmw_implementation)
find_package("${rmw_implementation}" REQUIRED)
get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")

foreach(typesupport_impl ${typesupport_impls})
rosidl_target_interfaces(obstacle_extractor
  ${PROJECT_NAME} ${typesupport_impl}
)
rosidl_target_interfaces(obstacle_tracker
  ${PROJECT_NAME} ${typesupport_impl}
)
endforeach()

