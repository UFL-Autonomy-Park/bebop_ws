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

    # Namespace definition
    robot_ns = 'bebop104'

    return LaunchDescription([
        Node(
            package='mocap_filters',
            executable='filters_node',
            name='mocap_filters',
            namespace=robot_ns,
            parameters=[config],
            output='screen',
            # Remap input if necessary, otherwise use param in yaml
            # remappings=[('pose', '/vrpn_mocap/bebop104/pose')] 
        )
    ])