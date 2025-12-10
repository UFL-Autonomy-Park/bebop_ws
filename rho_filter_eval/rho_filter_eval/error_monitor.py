import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Point
from visualization_msgs.msg import Marker
import message_filters

class ErrorMonitor(Node):
    def __init__(self):
        super().__init__('error_monitor')
        
        # Publishers
        self.pub_err = self.create_publisher(TwistStamped, 'error_twist', 10)
        self.pub_marker = self.create_publisher(Marker, 'error_marker', 10)

        # Subscribers
        # We use message_filters to ensure we compare messages from the same time
        self.sub_ref = message_filters.Subscriber(self, TwistStamped, 'reference_twist')
        self.sub_est = message_filters.Subscriber(self, TwistStamped, 'estimated_twist')
        
        # Synchronizer: slop is the max delay (in seconds) allowed between messages
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_ref, self.sub_est], queue_size=10, slop=0.05
        )
        self.ts.registerCallback(self.callback)

    def callback(self, ref_msg, est_msg):
        # Compute Error: e = v_ref - v_est
        err_msg = TwistStamped()
        err_msg.header = ref_msg.header
        
        e_x = ref_msg.twist.linear.x - est_msg.twist.linear.x
        e_y = ref_msg.twist.linear.y - est_msg.twist.linear.y
        e_z = ref_msg.twist.linear.z - est_msg.twist.linear.z
        
        err_msg.twist.linear.x = e_x
        err_msg.twist.linear.y = e_y
        err_msg.twist.linear.z = e_z
        
        self.pub_err.publish(err_msg)
        self.publish_viz_marker(e_x, e_y, e_z, ref_msg.header)

    def publish_viz_marker(self, x, y, z, header):
        # Visualizes the error vector in RViz
        marker = Marker()
        marker.header = header
        marker.ns = "error_vector"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Arrow start (0,0,0) and end (error vector)
        p_start = Point(x=0.0, y=0.0, z=0.0)
        p_end = Point(x=x, y=y, z=z)
        marker.points = [p_start, p_end]
        
        # Scale: shaft diameter, head diameter, head length
        marker.scale.x = 0.05 
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        
        marker.color.a = 1.0
        marker.color.r = 1.0  # Red for error
        
        self.pub_marker.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = ErrorMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()