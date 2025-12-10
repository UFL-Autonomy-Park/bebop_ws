import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
import numpy as np

# --- Signal Generator Function ---
# Edit this function to change the signal.
# Returns: (position_np_array, velocity_np_array)
def generate_signal(t_sec):
    # Example: Simple circular motion
    r = 2.0
    omega = 1.0
    
    x = r * np.cos(omega * t_sec)
    y = r * np.sin(omega * t_sec)
    z = 1.0
    
    vx = -r * omega * np.sin(omega * t_sec)
    vy = r * omega * np.cos(omega * t_sec)
    vz = 0.0
    
    pos = np.array([x, y, z])
    vel = np.array([vx, vy, vz])
    
    return pos, vel

class SignalPublisher(Node):
    def __init__(self):
        super().__init__('signal_publisher')
        
        # Publishers
        self.pub_pose = self.create_publisher(PoseStamped, 'reference_pose', 10)
        self.pub_twist = self.create_publisher(TwistStamped, 'reference_twist', 10)
        
        # Timer (120 Hz)
        self.dt = 1.0 / 120.0
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
        self.start_time = self.get_clock().now().nanoseconds / 1e9

    def timer_callback(self):
        now = self.get_clock().now()
        t_sec = (now.nanoseconds / 1e9) - self.start_time
        
        # Get signal data
        pos, vel = generate_signal(t_sec)
        
        # Populate PoseStamped
        msg_pose = PoseStamped()
        msg_pose.header.stamp = now.to_msg()
        msg_pose.header.frame_id = "map"
        msg_pose.pose.position.x = pos[0]
        msg_pose.pose.position.y = pos[1]
        msg_pose.pose.position.z = pos[2]
        # Note: Orientation is left as identity (0,0,0,1) for this example
        
        # Populate TwistStamped
        msg_twist = TwistStamped()
        msg_twist.header.stamp = now.to_msg()
        msg_twist.header.frame_id = "map"
        msg_twist.twist.linear.x = vel[0]
        msg_twist.twist.linear.y = vel[1]
        msg_twist.twist.linear.z = vel[2]
        
        self.pub_pose.publish(msg_pose)
        self.pub_twist.publish(msg_twist)

def main(args=None):
    rclpy.init(args=args)
    node = SignalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()