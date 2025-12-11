#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Int32 

class TrajectoryTracking(Node):
    def __init__(self):
        # Initialize the parent class with a node name
        super().__init__('figure_eight_node')

        # Subscriber
        self.pose_sub_ = self.create_subscription(
            PoseStamped,
            '/vrpn_mocap/bebop104/pose',
            self.pose_callback,
            qos_profile_sensor_data
        )
        self.mode_sub_ = self.create_subscription(
            Int32,
            '/bebop104/mode',
            self.mode_callback,
            10
        )

        # Publisher
        self.control_pub_ = self.create_publisher(
            Twist,
            '/bebop104/cmd_vel_des',
            10
        )
        self.trajectory_pub_ = self.create_publisher(
            PoseStamped,
            '/bebop104/trajectory_setpoint',
            10
        )     
        
        # Log
        self.get_logger().info('Figure eight trajectory tracking node has been initialized.')

        # Declare params
        self.declare_parameter('vert_offset', 0.5) # meters
        self.declare_parameter('update_rate', 50.0) # Hz
        self.declare_parameter('tracking_kp', 1.0)
        self.declare_parameter('heading_kp', 1.0)
        # Get params
        self.vert_offset = self.get_parameter('vert_offset').get_parameter_value().double_value
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        self.tracking_kp = self.get_parameter('tracking_kp').get_parameter_value().double_value
        self.heading_kp = self.get_parameter('heading_kp').get_parameter_value().double_value

        # Timer
        self.timer = self.create_timer(1.0/self.update_rate, self.timer_callback) 

        # Bebop state
        self.bebop_pose = PoseStamped()
        self.bebop_mode = int(0) # 0: teleop, 1: offboard
        # Desired Trajectory
        self.step = 0.0
        self.trajectory_setpoint = np.zeros(3) # world frame (x,y,z)
        # Output
        self.control_input = Twist()

    def timer_callback(self):
        if self.bebop_mode != 1:
            return # Only run in offboard mode
        elif self.bebop_mode == 1:
            # Compute desired trajectory (figure eight, constant altitude)
            self.trajectory_setpoint[0] = -0.5 * np.sin(2 * self.step)
            self.trajectory_setpoint[1] = self.vert_offset
            self.trajectory_setpoint[2] = 2 * np.sin(self.step)
            # Heading is tangent to trajectory
            traj_deriv_x = -1.0 * np.cos(2 * self.step)
            traj_deriv_z = 2.0 * np.cos(self.step)
            yaw_setpoint = np.arctan2(traj_deriv_z, traj_deriv_x)
            # Increment step
            self.step += 1.0/self.update_rate 
            
            # Get current yaw from pose
            q = self.bebop_pose.pose.orientation
            r_world = Rotation.from_quat([q.x, q.y, q.z, q.w])
            r_enu_correction = Rotation.from_euler('x', 90, degrees=True) # To ENU
            r_enu = r_enu_correction * r_world
            current_roll, current_pitch, current_yaw = r_enu.as_euler('xyz', degrees=False)

            # TEMP: Drive Bebop to fixed point
            self.trajectory_setpoint[0] = 0.0
            self.trajectory_setpoint[1] = self.vert_offset
            self.trajectory_setpoint[2] = 1.0

            # Compute control input (P controller)
            err_x = self.trajectory_setpoint[0] - self.bebop_pose.pose.position.x
            err_y = self.trajectory_setpoint[1] - self.bebop_pose.pose.position.y
            err_z = self.trajectory_setpoint[2] - self.bebop_pose.pose.position.z
            err_yaw = yaw_setpoint - current_yaw
            err_yaw = (err_yaw + np.pi) % (2 * np.pi) - np.pi # Wrap to [-pi, pi]
            self.control_input.linear.x = self.tracking_kp * err_x
            self.control_input.linear.y = self.tracking_kp * err_y
            self.control_input.linear.z = self.tracking_kp * err_z
            # self.control_input.angular.y = self.heading_kp * err_yaw

            # TEMP: Convert control input to body frame
            # control_input_world = np.zeros(3)
            # control_input_world[0] = self.tracking_kp * err_x
            # control_input_world[1] = self.tracking_kp * err_y
            # control_input_world[2] = self.tracking_kp * err_z
            # control_input_enu = r_enu_correction.apply(control_input_world)
            # r_yaw_correction = Rotation.from_euler('z', -current_yaw, degrees=False)
            # control_input_body = r_yaw_correction.apply(control_input_enu)
            # # clamp vals to [-1,1]
            # control_input_body = np.clip(control_input_body, -1.0, 1.0)
            # self.control_input.linear.x = control_input_body[0]
            # self.control_input.linear.y = control_input_body[1]
            # self.control_input.linear.z = control_input_body[2]
        

            # Publish control input
            self.control_pub_.publish(self.control_input)
            # Publish trajectory setpoint
            traj_msg = PoseStamped()
            traj_msg.header.stamp = self.get_clock().now().to_msg()
            traj_msg.header.frame_id = 'world'
            traj_msg.pose.position.x = self.trajectory_setpoint[0]
            traj_msg.pose.position.y = self.trajectory_setpoint[1]
            traj_msg.pose.position.z = self.trajectory_setpoint[2]
            r = Rotation.from_euler('y', -yaw_setpoint, degrees=False)
            qx, qy, qz, qw = r.as_quat()
            traj_msg.pose.orientation.x = qx
            traj_msg.pose.orientation.y = qy
            traj_msg.pose.orientation.z = qz
            traj_msg.pose.orientation.w = qw
            self.trajectory_pub_.publish(traj_msg)
        else:
            self.get_logger().warn('Bebop mode is not recognized. Shutting down node')
            # Kill node
            rclpy.shutdown()

    def pose_callback(self, msg):
        # get pose
        self.bebop_pose = msg

    def mode_callback(self, msg):
        # get mode
        self.bebop_mode = msg.data


def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)

    # Create the node
    node = TrajectoryTracking()

    # Spin
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass # Allow Ctrl+C to exit 
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()