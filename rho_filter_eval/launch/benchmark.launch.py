from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Signal Generator (Python)
        # Publishes: /reference_pose, /reference_twist
        Node(
            package='rho_filter_eval',
            executable='signal_publisher',
            name='signal_publisher',
            output='screen'
        ),

        # 2. Your Estimation Filter (C++)
        Node(
            package='rho_filter',
            executable='rho_filter_node',
            name='rho_filter',
            output='screen',
            remappings=[
                ('pose_input_topic', '/reference_pose'), 
                ('twist_output_topic', '/estimated_twist')
            ]
        ),

        # 3. Error Monitor (Python)
        # Subscribes: /reference_twist, /estimated_twist
        # Publishes: /error_twist, /error_marker
        Node(
            package='rho_filter_eval',
            executable='error_monitor',
            name='error_monitor',
            output='screen',
            # No remapping needed if topics match the lines above
        ),

        # 4. RQT Plot (Visualizes X, Y, Z errors over time)
        Node(
            package='rqt_plot',
            executable='rqt_plot',
            name='rqt_plot',
            arguments=[
                '/error_twist/twist/linear/x',
                '/error_twist/twist/linear/y',
                '/error_twist/twist/linear/z'
            ]
        ),

        # 5. RViz2 (Visualizes the 3D error arrow)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])