# Description:
# This node is responsible for:
# - publishing the transform between the UWB odometry based on the UWB Tag position
# - publishing the transform between the robot world frame (odom/map) and UWB coordinate frame
# The initial UWB position is the origin of the robot world frame
# The orientation of this system can be set based on:
# - initial robot orientation to manually align the robot world frame with arbitrary UWB coordinate frame
# - imu with mag sensor to automatically align the robot world frame with UWB coordinate frame which follows ENU convention


import rclpy   
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu 

from geometry_msgs.msg import (
    PointStamped, 
    Pose, 
    Transform, 
    TransformStamped, 
    Vector3, 
    Quaternion,
    Point
)

from tf2_ros import (
    Buffer, 
    TransformListener, 
    TransformBroadcaster, 
    TransformException
)

import tf2_geometry_msgs
from scipy.spatial.transform import Rotation as R
import numpy as np



class NavUWBTransformNode(Node):
    
    input_odom_topic = '/odometry/filtered'
    imu_topic = '/imu/data'
    uwb_tag_topic = '/uwb_tag_point/kf_spikes_filtered'
    uwb_odom_topic = '/odometry/uwb'
    
    def __init__(self):
        super().__init__('nav_uwb_transform_node')
        
        # declare parameters
        self.declare_parameter('yaw_offset', 0.0)
        self.declare_parameter('magnetic_declination', 0.0)
        self.declare_parameter('use_odometry_yaw', True)
        # self.declare_parameter('world_frame_id', 'map')
        self.declare_parameter('uwb_frame_id', 'uwb')
        self.declare_parameter('uwb_tag_frame_id', 'uwb_tag_link')
        # self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_tf_uwb_as_parent', False)
        self.declare_parameter('zero_altitude', True)
        
        # get parameters
        self.yaw_offset = self.get_parameter('yaw_offset').value
        self.magnetic_declination = self.get_parameter('magnetic_declination').value
        self.use_odometry_yaw = self.get_parameter('use_odometry_yaw').value
        self.uwb_frame_id = self.get_parameter('uwb_frame_id').value
        self.uwb_tag_frame_id = self.get_parameter('uwb_tag_frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.publish_tf_uwb_as_parent = self.get_parameter('publish_tf_uwb_as_parent').value
        self.zero_altitude = self.get_parameter('zero_altitude').value
        
        self.world_frame_id = None
        self.base_frame_id = None
        
        self.transform_good_ = False
        self.uwb_updated_ = False
        self.odom_updated_ = False  
        
        
        self.lastest_uwb_position_ = PointStamped()
        self.initial_uwb_position_ = None # initial uwb position used as world origin
        
        self.lastest_uwb_orientation_ = None # orientation of robot in UWB frame (based on odometry or imu) for publishing UWB odometry
        
        self.transform_uwb_orientation_ = None # initial orientation of robot in UWB frame (based on odometry or imu) which is used to compute transform
        self.transform_world_pose_ = None # initial position of robot in world frame which is used to compute transform
        
        self.world_to_uwb_frame_transform_ = Transform()
        self.uwb_to_world_frame_transform_ = Transform()
        
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        
        self.odom_sub = self.create_subscription(Odometry, self.input_odom_topic, self.odom_callback, 10)
        
        if not self.use_odometry_yaw:
            self.imu_sub = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)
        
        
        self.uwb_tag_postion_sub = self.create_subscription(PointStamped, self.uwb_tag_topic, self.uwb_tag_callback, 10)
        
        self.uwb_odom_pub = self.create_publisher(Odometry, self.uwb_odom_topic, 10)
        self.uwb_transform_timer = self.create_timer(1.0, self.uwb_transform_callback)
        
        self.get_logger().info('Nav UWB Transform Node has been initialized')
        
    
    def uwb_tag_callback(self, msg):
        if self.uwb_tag_frame_id != msg.header.frame_id:
            self.get_logger().warn('UWB tag frame id %s does not match expected frame id %s' % (msg.header.frame_id, self.uwb_tag_frame_id))
            return
        
        if (not self.transform_good_ and
            self.initial_uwb_position_ is None):
            self.get_logger().info('Initial UWB position set to %s' % msg.point)
            self.initial_uwb_position_ = msg
            
        self.lastest_uwb_position_ = msg
        self.uwb_updated_ = True
        
    def imu_callback(self, msg):
        #TODO: later
        pass
        
    def odom_callback(self, msg):
        
        # get world frame id and base frame id from input odometry message
        self.world_frame_id = msg.header.frame_id
        self.base_frame_id = msg.child_frame_id
        
        # get initial robot position in world frame for computing transform
        if not self.transform_good_ and self.transform_world_pose_ is None:
            
            #TODO: comment
            self.transform_world_pose_ = msg.pose.pose
            if self.use_odometry_yaw:
                self.transform_uwb_orientation_ = msg.pose.pose.orientation
                
                # adjust quaternion based on yaw offset and magnetic declination
                roll, pitch, yaw = R.from_quat([self.transform_uwb_orientation_.x,
                                                self.transform_uwb_orientation_.y,
                                                self.transform_uwb_orientation_.z,
                                                self.transform_uwb_orientation_.w]).as_euler('xyz')
                yaw += self.yaw_offset + self.magnetic_declination
                self.transform_uwb_orientation_ = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
                
                # convert back to quaternion message
                self.transform_uwb_orientation_ = Quaternion(x=self.transform_uwb_orientation_[0],
                                                             y=self.transform_uwb_orientation_[1],
                                                             z=self.transform_uwb_orientation_[2],
                                                             w=self.transform_uwb_orientation_[3])
        
        # use odometry yaw for UWB odometry if required
        if self.use_odometry_yaw:
            self.lastest_uwb_orientation_ = msg.pose.pose.orientation
        
                
    def uwb_transform_callback(self):
        if not self.transform_good_:
            
            self.compute_transform()
        else:
            uwb_odom = self.prepare_uwb_odom(self.lastest_uwb_position_)
            self.uwb_odom_pub.publish(uwb_odom)
            
    def prepare_uwb_odom(self, uwb_position: PointStamped):
        uwb_odom = None
        
        if (self.transform_good_ and self.uwb_updated_):
            uwb_odom = Odometry()
            uwb_odom.header.stamp = self.get_clock().now().to_msg()
            uwb_odom.header.frame_id = self.world_frame_id
            uwb_odom.child_frame_id = ""
            
            uwb_odom.pose.pose.position = uwb_position.point
            uwb_odom.pose.pose.orientation.x = 0.0
            uwb_odom.pose.pose.orientation.y = 0.0
            uwb_odom.pose.pose.orientation.z = 0.0
            uwb_odom.pose.pose.orientation.w = 1.0
            
            # convert uwb position to world frame
            uwb_pose_in_uwb_frame_mat = self.vector3_quarternion_to_homogeneous_matrix(uwb_position.point, uwb_odom.pose.pose.orientation)
            world_to_uwb_frame_transform_mat = self.vector3_quarternion_to_homogeneous_matrix(
                self.world_to_uwb_frame_transform_.translation, self.world_to_uwb_frame_transform_.rotation)
            uwb_pose_in_world_frame_mat = np.dot(world_to_uwb_frame_transform_mat, uwb_pose_in_uwb_frame_mat)
            
            _v3, _quad = self.homogeneous_matrix_to_vector3_quaternion(uwb_pose_in_world_frame_mat)
            uwb_odom.pose.pose.position.x = _v3.x
            uwb_odom.pose.pose.position.y = _v3.y
            uwb_odom.pose.pose.position.z = _v3.z
            uwb_odom.pose.pose.orientation = _quad
            
            # offset uwb position to robot origin and set orientation
            uwb_odom.pose.pose = self.get_robot_origin_uwb_pose(uwb_odom.pose.pose.position)
            
            if self.zero_altitude:
                uwb_odom.pose.pose.position.z = 0.0
            
            # publish uwb odometry
            self.uwb_odom_pub.publish(uwb_odom)
        
        return uwb_odom
    
    def compute_transform(self):
        # compute transform between robot world frame and UWB frame
        
        if (not self.transform_good_ and
            self.initial_uwb_position_ is not None and
            self.transform_uwb_orientation_ is not None
            ):
            
            self.get_logger().info('Computing transform between %s and %s' % (self.world_frame_id, self.uwb_frame_id))
            
            # get UWB pose of robot origin from initial UWB position
            initial_robot_origin_uwb_pose = self.get_robot_origin_uwb_pose(self.initial_uwb_position_.point)
            if initial_robot_origin_uwb_pose is None:
                return
            
            initial_robot_origin_world_pose = self.transform_world_pose_
            
            # compute transform from robot world frame to UWB frame and vice versa
            uwb_pose_mat = self.vector3_quarternion_to_homogeneous_matrix(
                initial_robot_origin_uwb_pose.position, initial_robot_origin_uwb_pose.orientation)
            uwb_pose_mat_inv = np.linalg.inv(uwb_pose_mat)
            world_pose_mat = self.vector3_quarternion_to_homogeneous_matrix(
                initial_robot_origin_world_pose.position, initial_robot_origin_world_pose.orientation)
            
            world_to_uwb_frame_transform_mat = np.dot(uwb_pose_mat_inv, world_pose_mat)
            uwb_to_world_frame_transform_mat = np.linalg.inv(world_to_uwb_frame_transform_mat)
            
            # convert homogeneous matrix to transform  
            _v3, _quad = self.homogeneous_matrix_to_vector3_quaternion(world_to_uwb_frame_transform_mat)
            self.world_to_uwb_frame_transform_.translation = _v3
            self.world_to_uwb_frame_transform_.rotation = _quad
            
            _v3, _quad = self.homogeneous_matrix_to_vector3_quaternion(uwb_to_world_frame_transform_mat)
            self.uwb_to_world_frame_transform_.translation = _v3
            self.uwb_to_world_frame_transform_.rotation = _quad
            
            self.transform_good_ = True

            self.get_logger().info(f'Computed transform between {self.world_frame_id} and {self.uwb_frame_id} : {self.world_to_uwb_frame_transform_}')
            
            if self.publish_tf:
                pub_transform = TransformStamped()
                pub_transform.header.stamp = self.get_clock().now().to_msg()
                
                if self.publish_tf_uwb_as_parent:
                    pub_transform.header.frame_id = self.uwb_frame_id
                    pub_transform.child_frame_id = self.world_frame_id
                    pub_transform.transform = self.uwb_to_world_frame_transform_
                else:
                    pub_transform.header.frame_id = self.world_frame_id
                    pub_transform.child_frame_id = self.uwb_frame_id
                    pub_transform.transform = self.world_to_uwb_frame_transform_
                    
                if self.zero_altitude:
                    pub_transform.transform.translation.z = 0.0
                
                self.tf_broadcaster.sendTransform(pub_transform)
                
                self.get_logger().info(f'Published transform between {pub_transform.header.frame_id} and {pub_transform.child_frame_id}')
                
                
                
    
    def vector3_quarternion_to_homogeneous_matrix(self, vector3, quaternion):
        # Create rotation matrix from quaternion
        rotation = R.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        rotation_matrix = rotation.as_matrix()

        # Create translation vector
        translation = np.array([vector3.x, vector3.y, vector3.z])

        # Construct the homogeneous transformation matrix
        homogeneous_matrix = np.block([
            [rotation_matrix, translation[:, np.newaxis]],  # Add translation as a column
            [np.zeros((1, 3)), np.array([[1]])]  # Add the bottom row [0, 0, 0, 1]
        ])

        return homogeneous_matrix

    def homogeneous_matrix_to_vector3_quaternion(self, homogeneous_matrix):
        # Extract rotation matrix
        rotation_matrix = homogeneous_matrix[:3, :3]

        # Extract translation vector
        translation = homogeneous_matrix[:3, 3]

        # Convert rotation matrix to quaternion
        rotation = R.from_matrix(rotation_matrix)
        quaternion_array = rotation.as_quat()  # [x, y, z, w]

        # Create vector3 message
        vector3 = Vector3()
        vector3.x = translation[0]
        vector3.y = translation[1]
        vector3.z = translation[2]

        # Create quaternion message
        quaternion = Quaternion()
        quaternion.x = quaternion_array[0]
        quaternion.y = quaternion_array[1]
        quaternion.z = quaternion_array[2]
        quaternion.w = quaternion_array[3]

        return vector3, quaternion
            
            
    
    def get_robot_origin_uwb_pose(self, uwb_position: Point) -> Pose:
        
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame_id, self.uwb_tag_frame_id, rclpy.time.Time(), rclpy.time.Duration(seconds=1.0))
            
            uwb_pose_without_orientation = Pose()
            uwb_pose_without_orientation.position.x = uwb_position.x
            uwb_pose_without_orientation.position.y = uwb_position.y
            uwb_pose_without_orientation.position.z = uwb_position.z
            
            # no orientation for uwb so it is identity quaternion
            uwb_pose_without_orientation.orientation.w = 1.0
            uwb_pose_without_orientation.orientation.x = 0.0
            uwb_pose_without_orientation.orientation.y = 0.0
            uwb_pose_without_orientation.orientation.z = 0.0
            
            # transform uwb position to robot origin
            robot_origin_uwb_pose = tf2_geometry_msgs.do_transform_pose(uwb_pose_without_orientation, transform)
            
            # set orientation of robot origin based on imu or odometry
            robot_origin_uwb_pose.orientation = self.transform_uwb_orientation_
            
            return robot_origin_uwb_pose
            
        
        except TransformException as e:
            self.get_logger().warn('Failed to get transform from %s to %s: %s' % (self.base_frame_id, self.uwb_tag_frame_id, e))
            return None
        
        
        
        
        



def main(args=None):
    rclpy.init(args=args)
    node = NavUWBTransformNode()
    rclpy.spin(node)
    rclpy.shutdown()
        
        
        
        
        