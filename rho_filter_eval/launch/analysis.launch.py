import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rho_filter'),
        'param',             # Directory name must match CMake install
        'rho_filter.yaml'    # Filename must match your file
    )

    return LaunchDescription([
        Node(
            package='rho_filter',
            executable='rho_filter_node',
            name='rho_filter_node',
            parameters=[config]
        ),
        Node(
            package='rho_filter_eval',
            executable='eval_node',
            name='eval_node'
        )
    ])