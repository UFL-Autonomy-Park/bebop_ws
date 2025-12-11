import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('mocap_filters'),
        'config',
        'filters.yaml'
    )

    return LaunchDescription([
        Node(
            package='mocap_filters',
            executable='levant_node',
            name='levant_node',
            namespace='bebop104',
            parameters=[config]
        ),
        Node(
            package='mocap_filters',
            executable='kalman_node',
            name='kalman_node',
            namespace='bebop104',
            parameters=[config]
        ),
        Node(
            package='mocap_filters',
            executable='rho_node',
            name='rho_node',
            namespace='bebop104',
            parameters=[config]
        )
    ])