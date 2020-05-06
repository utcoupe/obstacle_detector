cmake_minimum_required(VERSION 3.5)
project(processing_lidar_objects)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
# TODO C++17 ?
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

if(CMAKE_BUILD_TYPE MATCHES "DEBUG")
  add_compile_options(
    -g
    -Og
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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_kdl REQUIRED) # implicit dependency of tf2
# find_package(tf2_msgs REQUIRED)
# find_package(tf2_sensor_msgs REQUIRED)

# Include extra dependencies
find_package(Armadillo REQUIRED)

# Generate msgs
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/CircleObstacle.msg
  msg/Obstacles.msg
  msg/SegmentObstacle.msg
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
  ADD_LINTER_TESTS
)

include_directories(include)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

#
# Build Libs
#
add_library(scans_merger SHARED src/scans_merger.cpp)
target_compile_definitions(scans_merger
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(scans_merger
  rclcpp
  std_srvs
  laser_geometry
  sensor_msgs
  geometry_msgs
  rclcpp_components
  tf2
  tf2_ros
#  tf2_msgs
#  tf2_sensor_msgs
)
rclcpp_components_register_nodes(scans_merger "${PROJECT_NAME}::ScansMerger")
set(node_plugins "${node_plugins}${PROJECT_NAME}::ScansMerger;$<TARGET_FILE:scans_merger>\n")

add_library(obstacle_extractor SHARED src/obstacle_extractor.cpp)
target_compile_definitions(obstacle_extractor
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
target_link_libraries(obstacle_extractor
  ${ARMADILLO_LIBRARIES}
)
ament_target_dependencies(obstacle_extractor
  rclcpp
  std_srvs
  sensor_msgs
  geometry_msgs
  rclcpp_components
  tf2
  tf2_ros
  tf2_kdl
)
rclcpp_components_register_nodes(obstacle_extractor "${PROJECT_NAME}::ObstacleExtractor")
set(node_plugins "${node_plugins}${PROJECT_NAME}::ObstacleExtractor;$<TARGET_FILE:obstacle_extractor>\n")

add_library(obstacle_tracker SHARED src/obstacle_tracker.cpp)
target_compile_definitions(obstacle_tracker
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
target_link_libraries(obstacle_tracker
  ${ARMADILLO_LIBRARIES}
)
ament_target_dependencies(obstacle_tracker
  rclcpp
  std_srvs
  sensor_msgs
  geometry_msgs
  rclcpp_components
)
rclcpp_components_register_nodes(obstacle_tracker "${PROJECT_NAME}::ObstacleTracker")
set(node_plugins "${node_plugins}${PROJECT_NAME}::ObstacleTracker;$<TARGET_FILE:obstacle_tracker>\n")

add_library(obstacle_publisher SHARED src/obstacle_publisher.cpp)
target_compile_definitions(obstacle_publisher
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(obstacle_publisher
  rclcpp
  std_srvs
  sensor_msgs
  geometry_msgs
  rclcpp_components
)
rclcpp_components_register_nodes(obstacle_publisher "${PROJECT_NAME}::ObstaclePublisher")
set(node_plugins "${node_plugins}${PROJECT_NAME}::ObstaclePublisher;$<TARGET_FILE:obstacle_publisher>\n")

# Hacky way to depend on ROS msg/srv from the same package (as of ROS2 Eloquent)
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
rosidl_target_interfaces(obstacle_publisher
  ${PROJECT_NAME} ${typesupport_impl}
)
endforeach()


# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

#
# Build nodes
#
add_executable(scans_merger_node src/nodes/scans_merger_node.cpp)
target_link_libraries(scans_merger_node
  scans_merger
)
ament_target_dependencies(scans_merger_node rclcpp)


add_executable(obstacle_extractor_node src/nodes/obstacle_extractor_node.cpp)
target_link_libraries(obstacle_extractor_node
  obstacle_extractor
)
ament_target_dependencies(obstacle_extractor_node rclcpp)


add_executable(obstacle_tracker_node src/nodes/obstacle_tracker_node.cpp)
target_link_libraries(obstacle_tracker_node
  obstacle_tracker
)
ament_target_dependencies(obstacle_tracker_node rclcpp)


add_executable(obstacle_publisher_node src/nodes/obstacle_publisher_node.cpp)
target_link_libraries(obstacle_publisher_node
  obstacle_publisher
)
ament_target_dependencies(obstacle_publisher_node rclcpp)

#
# Install libraries
#

install(
  TARGETS
    scans_merger
    obstacle_extractor
    obstacle_tracker
    obstacle_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#
# Install executables
#

install(
  TARGETS
    scans_merger_node
    obstacle_extractor_node
    obstacle_tracker_node
    obstacle_publisher_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#
# Install config and launch folders
#

install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

#
# Extra install instructions
#

ament_package()
