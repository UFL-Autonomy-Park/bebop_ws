import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'bebop_trajectory_tracking'
    
    # Locate the .rviz file we just created
    rviz_config_path = os.path.join(
        get_package_share_directory(pkg_name),
        'param',
        'bebop_view.rviz'
    )

    return LaunchDescription([
        # Start RViz2 and pass the config file as an argument
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        ),
        Node(
            package='bebop_trajectory_tracking',
            executable='figure_eight',
            name='figure_eight_node',
            parameters=[os.path.join(get_package_share_directory('bebop_trajectory_tracking'), 'param', 'figure_eight_param.yaml')],
            output='screen'
        )
    ])