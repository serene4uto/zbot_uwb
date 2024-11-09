import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration

class GoalPublisherNode(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        
        # Initialize an action client for NavigateToPose
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.send_goal_to_nav2()

    def send_goal_to_nav2(self):
        # Wait for the action server to be available
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Waiting for 'navigate_to_pose' action server...")
        
        # Create a new goal message
        goal_msg = NavigateToPose.Goal()
        
        # Set the target position (x, y, z)
        goal_msg.pose.pose.position.x = 2.0
        goal_msg.pose.pose.position.y = 3.0
        goal_msg.pose.pose.position.z = 0.0
        
        # Set the target orientation (Quaternion) (e.g., facing forward)
        goal_msg.pose.pose.orientation.w = 1.0
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Send the goal and specify the callback functions for feedback and results
        self._action_client.wait_for_server()
        self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback).add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info(f"Feedback: {feedback.feedback}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected by server.")
            return

        self.get_logger().info("Goal accepted by server, waiting for result...")
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        if result:
            self.get_logger().info("Goal reached successfully!")
        else:
            self.get_logger().info("Goal failed.")

def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
