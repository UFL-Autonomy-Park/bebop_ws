import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def cfg_path(pkg, sub, filename):
    return os.path.join(get_package_share_directory(pkg), sub, filename)

def generate_launch_description():
    DIFFERENTIATOR_CFG = cfg_path('bebop_trajectory_tracking', 'param', 'differentiator_param.yaml')
    RVIZ_CFG = cfg_path('bebop_trajectory_tracking', 'param', 'bebop_view.rviz')

    BEBOP_NS = 'bebop104'

    # rosbag args
    SAVE_DATA = True
    OUTPUT_DIR = '/home/brandonfallin@ad.ufl.edu/bebop_ws/src/bebop_ws/experiment_data'
    MODE = 'rho' # rho, rushi_rho, dirty, levant, numeric
    BAG_NAME = f"differentiator_{MODE}_data"

    launch_actions = [
        Node(
            package='vrpn_mocap',
            executable='client_node',
            name='vrpn_mocap_client_node',
            namespace='vrpn_mocap',
            parameters=[DIFFERENTIATOR_CFG],
            output='screen'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[DIFFERENTIATOR_CFG],
            output='screen'
        ),
        Node(
            package='ros2_bebop_driver',
            executable='bebop_driver',
            name='bebop_driver_node',
            namespace=BEBOP_NS,
            parameters=[DIFFERENTIATOR_CFG],
            output='screen'
        ),
        Node(
            package='bebop_teleop',
            executable='bebop_teleop_node',
            name='bebop_teleop_node',
            namespace=BEBOP_NS,
            parameters=[DIFFERENTIATOR_CFG],
            output='screen'
        ),
        Node(
            package='mocap_filters',
            executable='filters_node',
            name='mocap_filters_node',
            namespace=BEBOP_NS,
            output='screen',
            parameters=[DIFFERENTIATOR_CFG]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', RVIZ_CFG]
        ),
        Node(
            package='bebop_trajectory_tracking',
            executable='figure_eight',
            name='figure_eight_node',
            namespace=BEBOP_NS,
            parameters=[DIFFERENTIATOR_CFG],
            output='screen'
        ),
        Node(
            package='bebop_control',
            executable='bebop_control_node',
            name='bebop_control_node',
            namespace=BEBOP_NS,
            parameters=[DIFFERENTIATOR_CFG],
            output='screen'
        ),
        Node(
            package='ncr_lab_viz',
            executable='ncr_lab_viz_node',
            name='ncr_lab_viz_node',
            namespace='ncr_lab_viz',
            parameters=[DIFFERENTIATOR_CFG],
            output='screen'
        ),
    ]

    if SAVE_DATA:
        launch_actions.append(
            ExecuteProcess(
                cmd = [
                    'bash', '-c',
                    f'ros2 bag record /{BEBOP_NS}/tracking_error /vrpn_mocap/{BEBOP_NS}/pose /{BEBOP_NS}/filtered_odom/rho /{BEBOP_NS}/filtered_odom/rushi_rho /{BEBOP_NS}/filtered_odom/dirty /{BEBOP_NS}/filtered_odom/levant /{BEBOP_NS}/filtered_odom/numeric /{BEBOP_NS}/cmd_vel /{BEBOP_NS}/trajectory_setpoint /{BEBOP_NS}/mode -o {OUTPUT_DIR}/{BAG_NAME}_$(date +%Y_%m_%d-%H_%M_%S)'
                ],
                output='screen'
            )
        )

    return LaunchDescription(launch_actions)