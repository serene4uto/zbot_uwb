import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, Transform, Quaternion, TransformStamped
import rclpy.time
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
import tf2_geometry_msgs
import scipy
from scipy.spatial.transform import Rotation as R
import numpy as np
from rclpy.time import Duration

def normalize_angles(angles):
    """
    Normalize an array of angles to the range [-pi, pi].
    
    Args:
        angles (numpy.ndarray): Array of angles in radians.
    
    Returns:
        numpy.ndarray: Array of normalized angles in radians.
    """
    return (angles + np.pi) % (2.0 * np.pi) - np.pi

def transform_to_matrix(transform):
    # Convert TransformStamped to a 4x4 transformation matrix
    translation = [
        transform.translation.x,
        transform.translation.y,
        transform.translation.z]
    rotation = [
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w]
    r = R.from_quat(rotation)
    matrix = np.identity(4)
    matrix[0:3, 0:3] = r.as_matrix()
    matrix[0:3, 3] = translation
    return matrix

def matrix_to_transform(matrix):
        # Convert 4x4 transformation matrix to geometry_msgs/Transform
        r = R.from_matrix(matrix[0:3, 0:3])
        quat = r.as_quat()
        transform = Transform()
        transform.translation.x = matrix[0, 3]
        transform.translation.y = matrix[1, 3]
        transform.translation.z = matrix[2, 3]
        transform.rotation.x = quat[0]
        transform.rotation.y = quat[1]
        transform.rotation.z = quat[2]
        transform.rotation.w = quat[3]
        return transform



class NavUwbTransformNode(Node):
    
    uwb_tag_point_topic = "/uwb_tag_point/kf_spikes_filtered"
    imu_topic = "/imu/data"
    odom_topic = "/odometry/filtered"
    
    # Whether we get the transform's yaw from the odometry or IMU source
    use_odometry_yaw_ = True
    
    # Whether or not we broadcast the Cartesian transform
    broadcast_cartesian_transform_ = True
    
    broadcast_cartesian_transform_as_parent_frame_ = False
    
    yaw_offset_ = 0.0
    
    # Whether or not to zero out the altitude of the UWB tag
    zero_altitude_ = True
    
    def __init__(self):
        super().__init__("nav_uwb_transform_node")
        self.get_logger().info("Nav UWB Transform Node Started")
        
        # whether or not we've computed a good heading
        self.transform_good_ = False
        
        self.uwb_updated_ = False
        self.uwb_update_time = None
        
        self.odom_updated_ = False
        self.odom_update_time = None
        
        
        
        #lastest imu orientation
        self.transform_orientation_ = Quaternion() 
        
        self.has_transform_odom_ = False
        self.has_transform_uwb_ = False
        self.has_transform_imu_ = False
        
        self.latest_world_pose_  = Transform()
        
        self.transform_cartesian_pose_ = Transform()
        self.latest_cartesian_pose_ = Transform()
        
        self.transform_world_pose_ = Transform()
        
        # Holds the cartesian->odom transform
        self.cartesian_world_transform_ = Transform()
        # Holds the inverse of the cartesian->odom transform for filterd UWB broacasts
        self.cartesian_world_trans_inverse_ = Transform()
        
        
        self.uwb_frame_id_ = None
        self.base_link_frame_id_ = None
        self.world_frame_id_ = None
        
        self.transform_timeout_ = Duration(nanoseconds=0)
        

        # Initialize transform broadcaster
        self.cartesian_broadcaster = TransformBroadcaster(self)
        
        
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
        
        # self.get_logger().info(f"w: {self.world_frame_id_}, b: {self.base_link_frame_id_}")
        
        if(not self.transform_good_):
            self.set_trans_for_odometry(msg)
        
        self.latest_world_pose_.translation.x = msg.pose.pose.position.x
        self.latest_world_pose_.translation.y = msg.pose.pose.position.y
        self.latest_world_pose_.translation.z = msg.pose.pose.position.z
        
        #TODO covariance ??
        
        self.odom_update_time = msg.header.stamp
        self.odom_updated_ = True
        
        
    def imu_callback(self, msg):
        # We need the base link frame id from the odometry message to get the transform
        # so we need to wait until we get that message before we can compute the transform
        if(self.has_transform_odom_):
            # This method only gets called if we don't yet have the
            # IMU data (the subscriber gets shut down once we compute
            # the transform), so we can assumed that every IMU message
            # that comes here is meant to be used for that purpose.
            
            self.transform_orientation_ = msg.orientation
            
            # Correct for the IMU's orientation w.r.t. base_link   
            try:         
                target_frame_trans = self.tf_buffer.lookup_transform(
                    self.base_link_frame_id_, msg.header.frame_id, msg.header.stamp, self.transform_timeout_)
            except TransformException as e:
                self.get_logger().error(f"Error looking up transform: {e}")
                return
            
            if(target_frame_trans):
                roll_offset, pitch_offset, yaw_offset = R.from_quat([
                    target_frame_trans.transform.rotation.x,
                    target_frame_trans.transform.rotation.y,
                    target_frame_trans.transform.rotation.z,
                    target_frame_trans.transform.rotation.w
                ]).as_euler('xyz', degrees=False)
                
                roll, pitch, yaw = R.from_quat([
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                    msg.orientation.w
                ]).as_euler('xyz', degrees=False)
                
                # Adjust RPY by subtracting offsets and normalize
                adjusted_roll = normalize_angles(roll - roll_offset)
                adjusted_pitch = normalize_angles(pitch - pitch_offset)
                adjusted_yaw = normalize_angles(yaw - yaw_offset)
                
                # Create rotations from adjusted RPY and yaw offset
                adjusted_rotation = R.from_euler('xyz', [adjusted_roll, adjusted_pitch, adjusted_yaw], degrees=False)
                yaw_rotation = R.from_euler('z', yaw_offset, degrees=False)

                # Combine the rotations
                new_rotation = yaw_rotation * adjusted_rotation
                
                # self.get_logger().info(f'Adjusted orientation: {new_rotation.as_euler("xyz", degrees=False)}')

                # Update transform_orientation_ with the new quaternion
                self.transform_orientation_.x = new_rotation.as_quat()[0]
                self.transform_orientation_.y = new_rotation.as_quat()[1]
                self.transform_orientation_.z = new_rotation.as_quat()[2]
                self.transform_orientation_.w = new_rotation.as_quat()[3]

                self.has_transform_imu_ = True
                # self.get_logger().info(f'Updated orientation: {self.transform_orientation_}')
            

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
        self.transform_cartesian_pose_.rotation.x = 0.0
        self.transform_cartesian_pose_.rotation.y = 0.0
        self.transform_cartesian_pose_.rotation.z = 0.0
        self.transform_cartesian_pose_.rotation.w = 1.0
        
        self.has_transform_uwb_ = True
        
        
    # def get_robot_origin_cartesian_pose(self, uwb_cartesian_pose, transform_time):
    #     """
    #         Given the pose of the uwb sensor in the Cartesian frame, removes the
    #         offset from the vehicle's centroid and returns the Cartesian-frame pose of said
    #         centroid.
    #     """
    #     robot_cartesian_pose = Transform()
        
    #     robot_cartesian_pose.rotation.x = 0.0
    #     robot_cartesian_pose.rotation.y = 0.0
    #     robot_cartesian_pose.rotation.z = 0.0
    #     robot_cartesian_pose.rotation.w = 1.0
        
    #     # Get linear offset from origin for UWB
    #     try:
    #         transform_offset = self.tf_buffer.lookup_transform(
    #             self.base_link_frame_id_, self.uwb_frame_id_, transform_time, self.transform_timeout_)
    #     except TransformException as e:
    #         if self.uwb_frame_id_:
    #             self.get_logger().error(
    #                 f"Unable to obtain {self.base_link_frame_id_} -> {self.uwb_frame_id_} transform. "
    #                 "Will assume navsat device is mounted at robot's origin")
    #         robot_cartesian_pose = uwb_cartesian_pose
    #         return robot_cartesian_pose

    #     cartesian_orientation = self.transform_orientation_
        
    #     roll, pitch, yaw = R.from_quat([
    #         cartesian_orientation.x,
    #         cartesian_orientation.y,
    #         cartesian_orientation.z,
    #         cartesian_orientation.w
    #     ]).as_euler('xyz', degrees=False)
        
    #     #TODO: check yaw if need offset : 
    #     yaw += self.yaw_offset_
    #     cartesian_rotation = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)
        
    #     # Rotate the UWB linear offset by the orientation
    #     origin = np.array([
    #         transform_offset.transform.translation.x,
    #         transform_offset.transform.translation.y,
    #         transform_offset.transform.translation.z
    #     ])
    #     rotated_origin = cartesian_rotation.apply(origin) 
        
    #     # Set the robot's origin in the Cartesian frame
    #     transform_offset.transform.translation.x = rotated_origin[0]
    #     transform_offset.transform.translation.y = rotated_origin[1]
    #     transform_offset.transform.translation.z = rotated_origin[2]
        
    #     # Zero out the rotation
    #     transform_offset.transform.rotation.x = 0.0
    #     transform_offset.transform.rotation.y = 0.0
    #     transform_offset.transform.rotation.z = 0.0
    #     transform_offset.transform.rotation.w = 1.0
        
    #     # Convert offset to homogeneous transformation matrix
    #     transform_offset_matrix = np.eye(4)
    #     transform_offset_matrix[0:3, 0:3] = R.from_quat([
    #         transform_offset.transform.rotation.x,
    #         transform_offset.transform.rotation.y,
    #         transform_offset.transform.rotation.z,
    #         transform_offset.transform.rotation.w
    #     ]).as_matrix()
        
    #     transform_offset_matrix[0:3, 3] = np.array([
    #         transform_offset.transform.translation.x,
    #         transform_offset.transform.translation.y,
    #         transform_offset.transform.translation.z
    #     ])
        
    #     # Compute inverse of the offset matrix
    #     transform_offset_matrix_inv = np.linalg.inv(transform_offset_matrix)
        
    #     # convert the UWB pose to a homogeneous transformation matrix
    #     uwb_quat = [
    #         uwb_cartesian_pose.rotation.x,
    #         uwb_cartesian_pose.rotation.y,
    #         uwb_cartesian_pose.rotation.z,
    #         uwb_cartesian_pose.rotation.w
    #     ]
        
    #     uwb_translation = [
    #         uwb_cartesian_pose.translation.x,
    #         uwb_cartesian_pose.translation.y,
    #         uwb_cartesian_pose.translation.z
    #     ]
        
    #     uwb_matrix = np.eye(4)
    #     uwb_matrix[0:3, 0:3] = R.from_quat(uwb_quat).as_matrix()
    #     uwb_matrix[0:3, 3] = uwb_translation
        
    #     # Apply the inverse offset to the UWB pose
    #     robot_matrix = np.dot(transform_offset_matrix_inv, uwb_matrix)
    #     robot_translation = robot_matrix[0:3, 3]
    #     robot_quat = R.from_matrix(robot_matrix[0:3, 0:3]).as_quat()
        
    #     # Set the robot's origin in the Cartesian frame
    #     robot_cartesian_pose.translation.x = robot_translation[0]
    #     robot_cartesian_pose.translation.y = robot_translation[1]
    #     robot_cartesian_pose.translation.z = robot_translation[2]
    #     robot_cartesian_pose.rotation.x = robot_quat[0]
    #     robot_cartesian_pose.rotation.y = robot_quat[1]
    #     robot_cartesian_pose.rotation.z = robot_quat[2]
    #     robot_cartesian_pose.rotation.w = robot_quat[3]
        
    #     return robot_cartesian_pose

    def get_robot_origin_cartesian_pose(self, uwb_cartesian_pose, transform_time):
        """
        Transforms the UWB tag position to the robot's base_link (origin) frame using tf2.

        Args:
            uwb_cartesian_pose (Transform): Position of the UWB tag in its original frame.
            transform_time (rclpy.time.Time): The timestamp for the transform.

        Returns:
            Transform: The UWB position transformed to the robot's origin (base_link) frame.
        """
        # Convert uwb_cartesian_pose to PointStamped or TransformStamped
        uwb_point = tf2_geometry_msgs.PointStamped()
        uwb_point.header.stamp = transform_time.to_msg()
        uwb_point.header.frame_id = self.uwb_frame_id_
        uwb_point.point.x = uwb_cartesian_pose.translation.x
        uwb_point.point.y = uwb_cartesian_pose.translation.y
        uwb_point.point.z = uwb_cartesian_pose.translation.z

        # Lookup the transform from UWB frame to base_link frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_link_frame_id_, self.uwb_frame_id_, transform_time, self.transform_timeout_)

            # Transform the UWB position to the robot's origin frame (base_link)
            robot_origin_point = tf2_geometry_msgs.do_transform_point(uwb_point, transform)

            # Create a Transform object with the transformed coordinates
            robot_cartesian_pose = Transform()
            robot_cartesian_pose.translation.x = robot_origin_point.point.x
            robot_cartesian_pose.translation.y = robot_origin_point.point.y
            robot_cartesian_pose.translation.z = robot_origin_point.point.z

            # Use the orientation from the transform (or keep the IMU-based orientation)
            robot_cartesian_pose.rotation = self.transform_orientation_

        except TransformException as e:
            self.get_logger().error(f"Unable to transform UWB position: {e}")
            return uwb_cartesian_pose  # Fallback to original UWB pose if transform fails

        return robot_cartesian_pose

    
    def cartesian_to_map(self, cartesian_pose):

        uwb_odom = Odometry()
        
        # Convert input Transform to a 4x4 matrix
        cartesian_pose_mat = transform_to_matrix(cartesian_pose)
        
        # Convert world transform to a 4x4 matrix
        cartesian_world_transform_mat = transform_to_matrix(self.cartesian_world_transform_)
        
        # Apply the world transform to the input pose
        transformed_cartesian_uwb_mat = np.dot(cartesian_world_transform_mat, cartesian_pose_mat)
        
        # Set the orientation to the identity
        transformed_cartesian_uwb_mat[0:3, 0:3] = np.eye(3)
        
        # Convert the transformed matrix back to a Transform
        transformed_cartesian_uwb = matrix_to_transform(transformed_cartesian_uwb_mat)
        
        # Populate Odometry message
        uwb_odom.header.frame_id = self.world_frame_id_
        uwb_odom.header.stamp = self.get_clock().now().to_msg()
        
        # Set pose
        uwb_odom.pose.pose.position.x = transformed_cartesian_uwb.translation.x
        uwb_odom.pose.pose.position.y = transformed_cartesian_uwb.translation.y
        uwb_odom.pose.pose.position.z = transformed_cartesian_uwb.translation.z
        uwb_odom.pose.pose.orientation = transformed_cartesian_uwb.rotation
        
        
        # Adjust Altitude if zero_altitude_ is set
        if self.zero_altitude_:
            uwb_odom.pose.pose.position.z = 0.0
            
        return uwb_odom

    

    def prepare_uwb_odom(self):
        
        uwb_odom = None
        
        if (self.transform_good_ and self.uwb_updated_ and self.odom_updated_):
            
            # First, convert the uwb pose from the uwb frame to the odom frame (world frame) using transform_world_pose_
            uwb_odom = self.cartesian_to_map(self.latest_cartesian_pose_)
            
            # self.get_logger().info("----------------------------------------------------")
            # self.get_logger().info(f"UWB Cartesian Pose before offset: {uwb_odom}")
            
            transformed_cartesian_uwb = Transform()
            transformed_cartesian_uwb.translation.x = uwb_odom.pose.pose.position.x
            transformed_cartesian_uwb.translation.y = uwb_odom.pose.pose.position.y
            transformed_cartesian_uwb.translation.z = uwb_odom.pose.pose.position.z
            transformed_cartesian_uwb.rotation = uwb_odom.pose.pose.orientation
            
            # Now we have the uwb location in the odom frame of the world
            # we need to offset it to the robot's origin
            # Want the pose of the vehicle origin, not the UWB
            transformed_cartesian_robot = self.get_robot_origin_cartesian_pose(
                transformed_cartesian_uwb, rclpy.time.Time(seconds=uwb_odom.header.stamp.sec, nanoseconds=uwb_odom.header.stamp.nanosec))
            
            # covariances ???
            
            # Update the pose in the odometry message
            uwb_odom.pose.pose.position.x = transformed_cartesian_robot.translation.x
            uwb_odom.pose.pose.position.y = transformed_cartesian_robot.translation.y
            uwb_odom.pose.pose.position.z = transformed_cartesian_robot.translation.z
            uwb_odom.pose.pose.orientation = transformed_cartesian_robot.rotation
            
            # Now that we have the uwb localtion in odom frame of the robot origin by offsetting using the transform 
            # between the base_link and the uwb frame
            
            # self.get_logger().info(f"UWB Cartesian Pose after offset: {uwb_odom}")
            
            # Adjust Altitude if zero_altitude_ is set
            if self.zero_altitude_:
                uwb_odom.pose.pose.position.z = 0.0
                
            self.uwb_updated_ = False
        
        return uwb_odom

        
        
    def uwb_transform_callback(self):
        if not self.transform_good_:
            self.compute_transform()
            
            if (self.transform_good_ and not self.use_odometry_yaw_):
                self.destroy_subscription(self.imu_sub)
        else:
            # prepare uwb odom and publish
            uwb_odom = self.prepare_uwb_odom()
            if uwb_odom:
                self.uwb_odom_pub.publish(uwb_odom)
                

    def compute_transform(self):
        
        # only do this if:
        # 1. We haven't computed the odom_frame->cartesian_frame transform before
        # 2. We've received the data we need
        if(not self.transform_good_ and self.has_transform_odom_ and self.has_transform_uwb_ and self.has_transform_imu_):
            self.get_logger().info("Computing Transform")
            
            # the UWB coordinate system pose we have is given at the location of the UWB tag on the robot.
            # We need to get UWB coordinate system pose of the robot's origin.
            
            transform_cartesian_pose_corrected = self.get_robot_origin_cartesian_pose(
                self.transform_cartesian_pose_, rclpy.time.Time())
            
            self.get_logger().info(f"UWB Cartesian Pose: {transform_cartesian_pose_corrected}")
            
            # Now, we now have this UWB pose in robot's origin frame and we have the input odometry pose of the robot in the world frame (odom)
            # We need to compute the transform between UWB pose and the odometry pose in the world frame ???
            # Then, we process the orientaion of this pose using imu orientation or yaw from input odometry
            imu_roll, imu_pitch, imu_yaw = R.from_quat([
                self.transform_orientation_.x,
                self.transform_orientation_.y,
                self.transform_orientation_.z,
                self.transform_orientation_.w
            ]).as_euler('xyz', degrees=False)
            
            imu_quat = R.from_euler('xyz', [0.0, 0.0, imu_yaw], degrees=False).as_quat()
            
            self.get_logger().info(f"IMU Quaternion: {imu_quat}")
            
            cartesian_pose_with_orientation = Transform()
            cartesian_pose_with_orientation.translation = transform_cartesian_pose_corrected.translation
            cartesian_pose_with_orientation.rotation.x = imu_quat[0]
            cartesian_pose_with_orientation.rotation.y = imu_quat[1]
            cartesian_pose_with_orientation.rotation.z = imu_quat[2]
            cartesian_pose_with_orientation.rotation.w = imu_quat[3]
            
            # Remove roll and pitch from odometry pose
            # Must be done because roll and pitch is removed from cartesian_pose_with_orientation

            odom_r = R.from_quat([
                self.transform_world_pose_.rotation.x,
                self.transform_world_pose_.rotation.y,
                self.transform_world_pose_.rotation.z,
                self.transform_world_pose_.rotation.w])
            _, _, odom_yaw = odom_r.as_euler('xyz')
            odom_quat = R.from_euler('xyz', [0.0, 0.0, odom_yaw]).as_quat()
            
            transform_world_pose_yaw_only = Transform()
            transform_world_pose_yaw_only.translation = self.transform_world_pose_.translation
            transform_world_pose_yaw_only.rotation.x = odom_quat[0]
            transform_world_pose_yaw_only.rotation.y = odom_quat[1]
            transform_world_pose_yaw_only.rotation.z = odom_quat[2]
            
            # Compute the transform between the input odometry pose and the UWB pose
            world_pose_mat = transform_to_matrix(transform_world_pose_yaw_only)
            cartesian_pose_mat = transform_to_matrix(cartesian_pose_with_orientation)
            cartesian_pose_inv = np.linalg.inv(cartesian_pose_mat)
            
            self.cartesian_world_transform_mat = np.dot(world_pose_mat, cartesian_pose_inv)
            self.cartesian_world_trans_inverse_mat = np.linalg.inv(self.cartesian_world_transform_mat)
            
            # Convert the transform to a Transform message
            self.cartesian_world_transform_ = matrix_to_transform(self.cartesian_world_transform_mat)
            self.cartesian_world_trans_inverse_ = matrix_to_transform(self.cartesian_world_trans_inverse_mat)
            
            self.transform_good_ = True
            
            # print the transform
            self.get_logger().info(f"Cartesian World Transform: {self.cartesian_world_transform_}")
            
            # Broadcast the trasnform if needed
            if self.broadcast_cartesian_transform_:
                cartesian_transform_stamped = TransformStamped()
                cartesian_transform_stamped.header.stamp = self.get_clock().now().to_msg()
                cartesian_frame_id = "uwb_reference"
                
                if self.broadcast_cartesian_transform_as_parent_frame_:
                    cartesian_transform_stamped.header.frame_id = cartesian_frame_id
                    cartesian_transform_stamped.child_frame_id = self.world_frame_id_
                    transform_to_send = matrix_to_transform(self.cartesian_world_trans_inverse_mat)
                else:
                    cartesian_transform_stamped.header.frame_id = self.world_frame_id_
                    cartesian_transform_stamped.child_frame_id = cartesian_frame_id
                    transform_to_send = matrix_to_transform(self.cartesian_world_transform_mat)
                    
                cartesian_transform_stamped.transform = transform_to_send
                
                if self.zero_altitude_:
                    cartesian_transform_stamped.transform.translation.z = 0.0
                
                self.cartesian_broadcaster.sendTransform(cartesian_transform_stamped)
                
                self.get_logger().info("Broadcasted Transform")
    
    
    def set_trans_for_odometry(self, msg):
        self.transform_world_pose_.translation.x = msg.pose.pose.position.x
        self.transform_world_pose_.translation.y = msg.pose.pose.position.y
        self.transform_world_pose_.translation.z = msg.pose.pose.position.z
        self.transform_world_pose_.rotation = msg.pose.pose.orientation
        self.has_transform_odom_ = True
        
        if (not self.transform_good_ and self.use_odometry_yaw_):
            self.get_logger().info(f"Using odometry yaw with pose: {self.transform_world_pose_}")
            imu = Imu()
            imu.orientation = msg.pose.pose.orientation
            imu.header.frame_id = msg.child_frame_id
            imu.header.stamp = msg.header.stamp
            self.imu_callback(imu)
            
            
            
            
    
            
            
            
            
        
    
        


def main(args=None):
    rclpy.init(args=args)
    
    node = NavUwbTransformNode()
    
    rclpy.spin(node)
    
    rclpy.shutdown()
        
        
        
        