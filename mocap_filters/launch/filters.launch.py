import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
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
            parameters=[config]
        )
    ])