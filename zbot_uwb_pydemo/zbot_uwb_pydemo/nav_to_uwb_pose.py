import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration

class NavToUwbPose(Node):
    def __init__(self):
        super().__init__('nav_to_uwb_pose')
        self.uwb_pose = None
            
            
def main(args=None):
    rclpy.init(args=args)
    nav_to_uwb_pose = NavToUwbPose()
    rclpy.spin(nav_to_uwb_pose)
    nav_to_uwb_pose.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
