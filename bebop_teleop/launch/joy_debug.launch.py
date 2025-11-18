import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    #Load park geometry parameters
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[os.path.join(get_package_share_directory('bebop_teleop'), 'param', 'joy_config.yaml')],
            output='screen'
        ),
        Node(
            package='bebop_teleop',
            executable='bebop_teleop_node',
            name='bebop_teleop_node',
            parameters=[
                os.path.join(get_package_share_directory('bebop_teleop'), 'param', 'xbox_controller.yaml'),
            ],
            output='screen'
        ),
    ])