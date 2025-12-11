import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to config
    config = os.path.join(
        get_package_share_directory('bebop_control'),
        'param',
        'bebop_param.yaml'
    )

    return LaunchDescription([
        Node(
            package='bebop_control',
            executable='bebop_control_node',
            name='bebop_control_node',
            output='screen',
            parameters=[config]
        )
    ])