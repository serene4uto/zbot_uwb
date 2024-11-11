import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped

class OdomToUWBReferenceNode(Node):
    def __init__(self):
        super().__init__('odom_to_uwb_reference_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.lookup_transform)  # Look up every second

    def lookup_transform(self):
        try:
            # Look up the transform from odom to uwb_reference
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'map', 'uwb_reference', rclpy.time.Time()
            )
            # Extract translation and rotation
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            self.get_logger().info(f"Translation: [{translation.x}, {translation.y}, {translation.z}]")
            self.get_logger().info(f"Rotation (Quaternion): [{rotation.x}, {rotation.y}, {rotation.z}, {rotation.w}]")
        except Exception as e:
            self.get_logger().warn(f"Could not transform 'map' to 'uwb_reference': {e}")

def main(args=None):
    rclpy.init(args=args)
    node = OdomToUWBReferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
