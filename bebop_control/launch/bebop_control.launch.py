import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to config
    bebop_control_config = os.path.join(
        get_package_share_directory('bebop_control'),
        'param',
        'bebop_param.yaml'
    )
    config_file_path = os.path.join(
        get_package_share_directory('mocap_filters'),
        'config',
        'filter_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='mocap_filters',
            executable='filters_node',
            name='mocap_filters_node',
            output='screen',
            parameters=[config_file_path],
            namespace='/bebop104',
        ),
        Node(
            package='bebop_control',
            executable='bebop_control_node',
            name='bebop_control_node',
            output='screen',
            parameters=[bebop_control_config]
        )
    ])
