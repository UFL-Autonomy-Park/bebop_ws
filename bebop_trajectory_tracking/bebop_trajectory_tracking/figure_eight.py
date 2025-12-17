#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped, Twist, Point
from visualization_msgs.msg import Marker
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
        self.error_pub_ = self.create_publisher(
            Twist,
            '/bebop104/tracking_error',
            10
        )
        self.bebop_sphere_pub_ = self.create_publisher(
            Marker,
            '/bebop104/marker',
            10
        )
        self.trajectory_buffer_pub_ = self.create_publisher(
            Marker,
            '/bebop104/trajectory_buffer',
            10
        )
        self.pose_buffer_pub_ = self.create_publisher(
            Marker,
            '/bebop104/pose_buffer',
            10
        )
        
        # Log
        self.get_logger().info('Figure eight trajectory tracking node has been initialized.')

        # Declare params
        self.declare_parameter('vert_offset', rclpy.Parameter.Type.DOUBLE) # meters
        self.declare_parameter('update_rate', rclpy.Parameter.Type.DOUBLE) # Hz
        self.declare_parameter('tracking_kp_lat', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('tracking_kp_vert', rclpy.Parameter.Type.DOUBLE)
        # Get params
        self.vert_offset = self.get_parameter('vert_offset').get_parameter_value().double_value
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        self.tracking_kp_lat = self.get_parameter('tracking_kp_lat').get_parameter_value().double_value
        self.tracking_kp_vert = self.get_parameter('tracking_kp_vert').get_parameter_value().double_value

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
        # Viz
        self.trajectory_buffer = deque(maxlen=250) # Keep last 250 points
        self.pose_buffer = deque(maxlen=250)

    def timer_callback(self):
        # Publish Bebop marker
        bebop_marker = Marker()
        bebop_marker.header.frame_id = self.bebop_pose.header.frame_id
        bebop_marker.header.stamp = self.get_clock().now().to_msg()
        bebop_marker.type = Marker.SPHERE
        bebop_marker.action = Marker.ADD
        bebop_marker.pose.position.x = self.bebop_pose.pose.position.x
        bebop_marker.pose.position.y = self.bebop_pose.pose.position.y
        bebop_marker.pose.position.z = self.bebop_pose.pose.position.z
        bebop_marker.scale.x = 0.3
        bebop_marker.scale.y = 0.3
        bebop_marker.scale.z = 0.3
        bebop_marker.color.r = 0.0
        bebop_marker.color.g = 1.0
        bebop_marker.color.b = 0.0
        bebop_marker.color.a = 1.0
        self.bebop_sphere_pub_.publish(bebop_marker)
        # Publish pose buffer
        new_pose_point = Point()
        new_pose_point.x = self.bebop_pose.pose.position.x
        new_pose_point.y = self.bebop_pose.pose.position.y
        new_pose_point.z = self.bebop_pose.pose.position.z
        self.pose_buffer.append(new_pose_point)
        pose_buffer_marker = Marker()
        pose_buffer_marker.header.frame_id = self.bebop_pose.header.frame_id
        pose_buffer_marker.header.stamp = self.get_clock().now().to_msg()
        pose_buffer_marker.type = Marker.LINE_STRIP
        pose_buffer_marker.action = Marker.ADD
        pose_buffer_marker.scale.x = 0.05
        pose_buffer_marker.color.r = 0.0
        pose_buffer_marker.color.g = 1.0
        pose_buffer_marker.color.b = 0.0
        pose_buffer_marker.color.a = 0.3
        pose_buffer_marker.points = list(self.pose_buffer)
        self.pose_buffer_pub_.publish(pose_buffer_marker)
        
        if self.bebop_mode != 1:
            return # Only run in offboard mode
        elif self.bebop_mode == 1:
            # Compute desired trajectory (figure eight, constant altitude)
            self.trajectory_setpoint[0] = -1.0 * np.sin(2 * self.step)
            self.trajectory_setpoint[1] = self.vert_offset
            self.trajectory_setpoint[2] = 3 * np.sin(self.step)
            # Heading is tangent to trajectory
            traj_deriv_x = -2.0 * np.cos(2 * self.step)
            traj_deriv_z = 3.0 * np.cos(self.step)
            yaw_setpoint = np.arctan2(traj_deriv_z, traj_deriv_x)
            # Increment step
            self.step += 1.0/(self.update_rate * 4.5)
            
            # Get current yaw from pose
            q = self.bebop_pose.pose.orientation
            r_world = Rotation.from_quat([q.x, q.y, q.z, q.w])
            r_enu_correction = Rotation.from_euler('x', 90, degrees=True) # To ENU
            r_enu = r_world.inv() * r_enu_correction 
            current_roll, current_pitch, current_yaw = r_enu.as_euler('xyz', degrees=False)

            # Compute control input (P controller)
            err_x = self.trajectory_setpoint[0] - self.bebop_pose.pose.position.x
            err_y = self.trajectory_setpoint[1] - self.bebop_pose.pose.position.y
            err_z = self.trajectory_setpoint[2] - self.bebop_pose.pose.position.z
            err_yaw = -(yaw_setpoint - current_yaw)
            err_yaw = (err_yaw + np.pi) % (2 * np.pi) - np.pi # Wrap to [-pi, pi]
            self.control_input.linear.x = self.tracking_kp_lat * err_x
            self.control_input.linear.y = self.tracking_kp_vert * err_y
            self.control_input.linear.z = self.tracking_kp_lat * err_z
            self.control_input.angular.y = err_yaw

            # Publish control input
            self.control_pub_.publish(self.control_input)
            # Publish trajectory setpoint
            traj_msg = PoseStamped()
            traj_msg.header.stamp = self.get_clock().now().to_msg()
            traj_msg.header.frame_id = 'odom'
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
            # Publish tracking error
            err_msg = Twist()
            err_msg.linear.x = err_x
            err_msg.linear.y = err_y
            err_msg.linear.z = err_z
            err_msg.angular.y = err_yaw
            self.error_pub_.publish(err_msg)

            
            # Publish desired trajectory buffer
            new_trajectory_point = Point()
            new_trajectory_point.x = self.trajectory_setpoint[0]
            new_trajectory_point.y = self.trajectory_setpoint[1]
            new_trajectory_point.z = self.trajectory_setpoint[2]
            self.trajectory_buffer.append(new_trajectory_point)
            trajectory_buffer_marker = Marker()
            trajectory_buffer_marker.header.frame_id = self.bebop_pose.header.frame_id
            trajectory_buffer_marker.header.stamp = self.get_clock().now().to_msg()
            trajectory_buffer_marker.type = Marker.LINE_STRIP
            trajectory_buffer_marker.action = Marker.ADD
            trajectory_buffer_marker.scale.x = 0.05
            trajectory_buffer_marker.color.r = 1.0
            trajectory_buffer_marker.color.g = 0.0
            trajectory_buffer_marker.color.b = 0.0
            trajectory_buffer_marker.color.a = 0.3
            trajectory_buffer_marker.points = list(self.trajectory_buffer)
            self.trajectory_buffer_pub_.publish(trajectory_buffer_marker)

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