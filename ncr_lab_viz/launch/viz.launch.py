import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    #Load park geometry parameters
    return LaunchDescription([
        Node(
            package='ncr_lab_viz',
            executable='ncr_lab_viz_node',
            name='ncr_lab_viz_node',
            namespace='ncr_lab_viz',
            parameters=[os.path.join(get_package_share_directory('ncr_lab_viz'), 'param', 'lab_geometry.yaml')],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('ncr_lab_viz'), 'rviz', 'ncr_lab.rviz')]
        )
    ])