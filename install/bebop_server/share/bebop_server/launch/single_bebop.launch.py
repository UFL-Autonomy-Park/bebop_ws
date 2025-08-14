import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    #Load park geometry parameters
    return LaunchDescription([
        Node(
            package='vrpn_mocap',
            executable='client_node',
            name='vrpn_mocap_client_node',
            namespace='vrpn_mocap',
            parameters=[
                os.path.join(get_package_share_directory('vrpn_mocap'), 'config', 'client.yaml'),
                {'server': '192.168.1.202'},
                {'port': 3883}
            ],
            output='screen'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[os.path.join(get_package_share_directory('bebop_teleop'), 'param', 'joy_config.yaml')],
            output='screen'
        ),
        Node(
            package='bebop_server',
            executable='bebop_server_node',
            name='bebop_server_node',
            parameters=[
                os.path.join(get_package_share_directory('bebop_server'), 'param', 'xbox_controller.yaml'),
            ],
            output='screen'
        ),
        Node(
            package='bebop_client',
            executable='bebop_client_node',
            namespace='bebop2',
            output='screen'
        ),
        Node(
            package='ros2_bebop_driver',
            executable='bebop_driver',
            name='bebop_driver_node',
            namespace='bebop2',
            parameters=[
                {'bebop_ip': '192.168.1.122'},
                {'camera_calibration_file': 'package://ros2_bebop_driver/config/bebop2_camera_calib.yaml'}
            ]
        )
    ])