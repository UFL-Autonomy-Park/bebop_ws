import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory('mocap_filters'),
        'param',
        'filter_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='mocap_filters',
            executable='filters_node',
            name='mocap_filters_node',
            namespace='bebop104',
            output='screen',
            parameters=[config_file_path],
        )
    ])