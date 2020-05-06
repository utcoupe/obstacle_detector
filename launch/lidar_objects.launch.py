#!/usr/bin/env python3

"""Launch all processing_lidar_objects nodes into a component container."""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = 'processing_lidar_objects'
NAMESPACE = f'/{PACKAGE_NAME}' # optional, can be blank

# File in config folder with all custom default parameters
PARAMETER_YAML_FILE = 'lidar_objects.param.yaml'

# As of ROS2 eloquent (and dashing), passing scoped yaml files to components does not work
# (See issue https://github.com/ros2/rclcpp/issues/715)
# However, yamls without scope (namespace, node name) still work.
PARAMETER_YAML_FILES = {
    'scans_merger': 'lidar_objects/scans_merger.param.yaml',
    'obstacle_extractor': 'lidar_objects/obstacle_extractor.param.yaml',
    'obstacle_tracker': 'lidar_objects/obstacle_tracker.param.yaml',
}

def generate_launch_description():
    """Generate launch description with multiple components."""
    
    config_folder = get_package_share_directory(PACKAGE_NAME) + '/config/'
    common_param_file = config_folder + PARAMETER_YAML_FILE

    # FIXME From ROS2 Foxy, node_name and node_namespace arguments
    # will become name and namespace and will warn about deprecation
    container = ComposableNodeContainer(
        # Creates the "main" programm that will be hosting the components
        node_name=f'{PACKAGE_NAME}_container',
        node_namespace=NAMESPACE,
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[
            # Lists the components to load and their specific parameters
            # See 'ros2 component types' for valid node_plugin names
            ComposableNode(
                package=PACKAGE_NAME,
                node_plugin=f'{PACKAGE_NAME}::ScansMerger',
                node_name='scans_merger',
                node_namespace=NAMESPACE,
                parameters=[
                    common_param_file,
                    config_folder + PARAMETER_YAML_FILES['scans_merger'],
                ],
            ),
            ComposableNode(
                package=PACKAGE_NAME,
                node_plugin=f'{PACKAGE_NAME}::ObstacleExtractor',
                node_namespace=NAMESPACE,
                node_name='obstacle_extractor',
                parameters=[
                    common_param_file,
                    config_folder + PARAMETER_YAML_FILES['obstacle_extractor'],
                ],
            ),
            ComposableNode(
                package=PACKAGE_NAME,
                node_plugin=f'{PACKAGE_NAME}::ObstacleTracker',
                node_namespace=NAMESPACE,
                node_name='obstacle_tracker',
                parameters=[
                    common_param_file,
                    config_folder + PARAMETER_YAML_FILES['obstacle_tracker'],
                ],
                remappings=[('tracked_obstacles', 'obstacles')],
            ),
        ],
        output='screen',
        parameters=[
            {'use_sim_time': False},
        ]
    )

    return launch.LaunchDescription([container])
