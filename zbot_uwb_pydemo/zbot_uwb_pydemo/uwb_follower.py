import rclpy
from rclpy.node import Node

import tf2_ros





class UwbFollowerNode(Node):
    def __init__(self):
        super().__init__('uwb_follower_node')
        self.get_logger().info("UwbFollowerNode init")
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        



def main(args=None):
    rclpy.init(args=args)
    uwb_follower_node = UwbFollowerNode()
    rclpy.spin(uwb_follower_node)
    rclpy.shutdown()