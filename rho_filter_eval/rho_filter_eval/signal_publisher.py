import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
import numpy as np

# --- Signal Generator: Surge and Stop ---
# Pattern: Surge forward for 3s, Stop for 3s, Repeat.
# Axis: Moves in global X only.
def generate_signal(t_total):
    cycle_duration = 6.0
    active_duration = 3.0
    v_max = 1.0  # m/s

    # Helper: angular frequency for the surge hump (1 - cos)
    omega = 2 * np.pi / active_duration
    
    # Calculate cycle counts and local time
    num_cycles = int(t_total // cycle_duration)
    tau = t_total % cycle_duration  # Time within current cycle

    # --- Velocity Calculation ---
    if tau < active_duration:
        # Smooth activation: v = (Vmax/2) * (1 - cos(omega*t))
        vx = (v_max / 2.0) * (1 - np.cos(omega * tau))
    else:
        # Stop phase
        vx = 0.0
    
    vy = 0.0
    vz = 0.0

    # --- Position Calculation (Analytical Integral) ---
    # Distance covered in one full active phase
    # Integral of (Vmax/2)*(1 - cos(wt)) from 0 to T_active is (Vmax/2)*T_active
    dist_per_cycle = (v_max / 2.0) * active_duration
    
    # Distance in current cycle
    if tau < active_duration:
        # Integral: (Vmax/2) * [t - (1/w)sin(wt)]
        current_dist = (v_max / 2.0) * (tau - (1/omega) * np.sin(omega * tau))
    else:
        # If in stop phase, we have completed the full active distance
        current_dist = dist_per_cycle

    x = (num_cycles * dist_per_cycle) + current_dist
    y = 0.0
    z = 1.0 # Hover at 1m

    return np.array([x, y, z]), np.array([vx, vy, vz])

class SignalPublisher(Node):
    def __init__(self):
        super().__init__('signal_publisher')
        self.pub_pose = self.create_publisher(PoseStamped, 'reference_pose', 10)
        self.pub_twist = self.create_publisher(TwistStamped, 'reference_twist', 10)
        self.timer = self.create_timer(1.0/120.0, self.timer_callback)
        self.start_time = self.get_clock().now().nanoseconds / 1e9

    def timer_callback(self):
        now = self.get_clock().now()
        t_sec = (now.nanoseconds / 1e9) - self.start_time
        
        pos, vel = generate_signal(t_sec)
        
        # Pose
        msg_pose = PoseStamped()
        msg_pose.header.stamp = now.to_msg()
        msg_pose.header.frame_id = "map"
        msg_pose.pose.position.x = pos[0]
        msg_pose.pose.position.y = pos[1]
        msg_pose.pose.position.z = pos[2]
        msg_pose.pose.orientation.w = 1.0 # Identity
        
        # Twist
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
    rclpy.spin(SignalPublisher())
    rclpy.shutdown()