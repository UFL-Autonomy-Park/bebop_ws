# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import TwistStamped

# class EvalNode(Node):
#     def __init__(self):
#         super().__init__('eval_node')
#         self.pub = self.create_publisher(TwistStamped, '/error_twist', 10)
#         self.sub_est = self.create_subscription(TwistStamped, '/estimated_twist', self.est_cb, 10)
#         self.sub_ref = self.create_subscription(TwistStamped, '/reference_twist', self.ref_cb, 10)
#         self.last_ref = None

#     def ref_cb(self, msg):
#         self.last_ref = msg

#     def est_cb(self, msg):
#         if self.last_ref is None:
#             return

#         error = TwistStamped()
#         error.header.stamp = self.get_clock().now().to_msg()
#         error.header.frame_id = msg.header.frame_id
        
#         # error = ref - est
#         error.twist.linear.x = self.last_ref.twist.linear.x - msg.twist.linear.x
#         error.twist.linear.y = self.last_ref.twist.linear.y - msg.twist.linear.y
#         error.twist.linear.z = self.last_ref.twist.linear.z - msg.twist.linear.z
        
#         self.pub.publish(error)

# def main(args=None):
#     rclpy.init(args=args)
#     rclpy.spin(EvalNode())
#     rclpy.shutdown()