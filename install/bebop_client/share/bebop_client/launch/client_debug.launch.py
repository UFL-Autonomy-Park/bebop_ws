import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    #Load park geometry parameters
    return LaunchDescription([
        Node(
            package='bebop_client',
            executable='bebop_client_node',
            namespace='bebop2',
            output='screen'
        ),
        Node(
            package='bebop_client',
            executable='bebop_client_node',
            namespace='bebop6',
            output='screen'
        ),
        Node(
            package='bebop_client',
            executable='bebop_client_node',
            namespace='bebop11',
            output='screen'
        )
    ])
    