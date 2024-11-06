import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, Transform
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy



class NavUwbTransformNode(Node):
    
    uwb_tag_point_topic = "/uwb_tag_point/kf"
    imu_topic = "/imu/data"
    odom_topic = "/odometry/filtered"
    
    # Whether we get the transform's yaw from the odometry or IMU source
    use_odometry_yaw_ = False
    
    def __init__(self):
        super().__init__("nav_uwb_transform_node")
        self.get_logger().info("Nav UWB Transform Node Started")
        
        # whether or not we've computed a good heading
        self.transform_good_ = False
        
        self.uwb_updated_ = False
        self.uwb_update_time = None
        
        self.odom_updated_ = False
        self.odom_update_time = None
        
        self.lastest_world_pose_ = None
        
        #lastest imu orientation
        self.transform_orientation_ = None
        
        self.has_transform_odom_ = False
        self.has_transform_uwb_ = False
        self.has_transform_imu_ = False
        
        
        self.transform_cartesian_pose_ = Transform()
        self.latest_cartesian_pose_ = Transform()
        
        self.transform_world_pose_ = Transform()
        
        
        self.uwb_frame_id_ = None
        self.base_link_frame_id_ = None
        self.world_frame_id_ = None
        
        self.transform_timeout_ = 0.0
        
        
        # Initialize the tf2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # self.custome_qos = QoSProfile(depth=10)
        # keep last msg in the queue
        custom_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        self.uwb_tag_point_sub = self.create_subscription(
            PointStamped, 
            self.uwb_tag_point_topic, 
            self.uwb_tag_point_callback, 
            custom_qos)
        
        self.imu_sub = self.create_subscription(
            Imu, 
            self.imu_topic, 
            self.imu_callback, 
            custom_qos)
        
        self.odom_sub = self.create_subscription(
            Odometry, 
            self.odom_topic, 
            self.odom_callback, 
            custom_qos)
        
        self.uwb_odom_pub = self.create_publisher(Odometry, "/odometry/uwb", 10)
        
        
        self.uwb_transform_timer = self.create_timer(1.0, self.uwb_transform_callback)
        
        
            
    def odom_callback(self, msg):
        self.world_frame_id_ = msg.header.frame_id
        self.base_link_frame_id_ = msg.child_frame_id
        
        self.get_logger().info(f"w: {self.world_frame_id_}, b: {self.base_link_frame_id_}")
        
        if(not self.transform_good_):
            self.set_trans_for_odometry(msg)
        
        self.lastest_world_pose_.translation.x = msg.pose.pose.position.x
        self.lastest_world_pose_.translation.y = msg.pose.pose.position.y
        self.lastest_world_pose_.translation.z = msg.pose.pose.position.z
        
        #TODO covariance ??
        
        self.odom_update_time = msg.header.stamp
        self.odom_updated_ = True
        
        
        
        

    def imu_callback(self, msg):
        # We need the base link frame id from the odometry message to get the transform
        # so we need to wait until we get that message before we can compute the transform
        if(self.has_transform_odom_):
            self.transform_orientation_ = msg.orientation
            
            # Correct for the IMU's orientation w.r.t. base_link            
            target_frame_trans = self.tf_buffer.lookup_transform(
                self.base_link_frame_id_, msg.header.frame_id, msg.header.stamp, self.transform_timeout_)
            
            if(target_frame_trans):
                #TODO
                pass
            
            
            


    def uwb_tag_point_callback(self, msg):
        # self.get_logger().info(f"Received UWB Tag Point: {msg.point.x}, {msg.point.y}, {msg.point.z}")
        self.uwb_frame_id_ = msg.header.frame_id
        
        if(not self.transform_good_):
            # if we haven't computed the transform yet, then store this msg as initial UWB data to use
            if( not self.has_transform_uwb_):
                self.set_trans_from_UWB(msg)
        
        self.latest_cartesian_pose_.translation.x = msg.point.x
        self.latest_cartesian_pose_.translation.y = msg.point.y
        self.latest_cartesian_pose_.translation.z = msg.point.z
        
        #TODO covariance ??
        
        self.uwb_updated_ = True
        self.uwb_update_time = msg.header.stamp
        
        
        
        
    def set_trans_from_UWB(self, msg):
        self.transform_cartesian_pose_.translation.x = msg.point.x
        self.transform_cartesian_pose_.translation.y = msg.point.y
        self.transform_cartesian_pose_.translation.z = msg.point.z
        self.transform_cartesian_pose_.rotation.x = 0
        self.transform_cartesian_pose_.rotation.y = 0
        self.transform_cartesian_pose_.rotation.z = 0
        self.transform_cartesian_pose_.rotation.w = 1
        
        self.has_transform_uwb_ = True
        
        
    def getRobotOriginCartesianPose(self, uwb_cartesian_pose, robot_cartesian_pose, transform_time):
        robot_cartesian_pose.rotation.x = 0
        robot_cartesian_pose.rotation.y = 0
        robot_cartesian_pose.rotation.z = 0
        robot_cartesian_pose.rotation.w = 1
        
        # Get linear offset from origin for UWB
        transform_stamped = self.tf_buffer.lookup_transform(
            self.base_link_frame_id_, self.uwb_frame_id_, transform_time, self.transform_timeout_)
        
        if(transform_stamped):
            #TODO
            pass
            
        
        
        
    def uwb_transform_callback(self):
        if not self.transform_good_:
            self.compute_transform()
        
        else:
            # prepare uwb odom and publish
            pass
            

    def compute_transform(self):
        
        # only do this if:
        # 1. We haven't computed the odom_frame->cartesian_frame transform before
        # 2. We've received the data we need
        if(not self.transform_good_ and self.has_transform_odom_ and self.has_transform_uwb_ and self.has_transform_imu_):
            self.get_logger().info("Computing Transform")
            
            # the UWB coordinate system pose we have is given at the location of the UWB tag on the robot.
            # We need to get UWB coordinate system pose of the robot's origin.
            
            self.getRobotOriginCartesianPose
            
    
    
    
    
    
    
    
    def set_trans_for_odometry(self, msg):
        self.transform_world_pose_.translation.x = msg.pose.pose.position.x
        self.transform_world_pose_.translation.y = msg.pose.pose.position.y
        self.transform_world_pose_.translation.z = msg.pose.pose.position.z
        self.transform_world_pose_.rotation = msg.pose.pose.orientation
        self.has_transform_odom_ = True
        
        if (not self.transform_good_ and self.use_odometry_yaw_):
            imu = Imu()
            imu.orientation = msg.pose.pose.orientation
            imu.header.frame_id = msg.header.child_frame_id
            imu.header.stamp = msg.header.stamp
            self.imu_callback(imu)
            
            
            
            
    
            
            
            
            
        
    
        


def main(args=None):
    rclpy.init(args=args)
    
    node = NavUwbTransformNode()
    
    rclpy.spin(node)
    
    rclpy.shutdown()
        
        
        
        