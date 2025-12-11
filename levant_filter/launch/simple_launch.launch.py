import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('levant_filter'),
        'param',
        'levant_filter.yaml'
    )

    return LaunchDescription([
        Node(
            package='levant_filter',
            executable='levant_node',
            name='levant_node',
            parameters=[config],
            output='screen'
        )
    ])