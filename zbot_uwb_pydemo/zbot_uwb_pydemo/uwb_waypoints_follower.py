import rclpy
from rclpy.node import Node

import tf2_ros

import tkinter as tk
from tkinter import filedialog
import yaml

from geometry_msgs.msg import PointStamped, Pose, PoseStamped

import tf2_geometry_msgs

from nav2_simple_commander.robot_navigator import BasicNavigator

from scipy.spatial.transform import Rotation as R


class UwbWaypointsFollowerNode(Node, tk.Tk):
    
    map_frame = 'map'
    uwb_ref_frame = 'uwb_reference'
    
    def __init__(self):
        tk.Tk.__init__(self)
        Node.__init__(self, 'uwb_waypoints_follower_node')
        
        self.navigator = BasicNavigator("basic_navigator")
        # wait for action client
        # self.navigator.waitUntilNav2Active(localizer='robot_localization')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.uwb_wps = None
        
        self.title("UWB Waypoint Follower GUI")
        self.resizable(False, False)

        self.load_label = tk.Label(self, text="File path:")
        self.load_textbox = tk.Entry(self, width=45)
        self.load_textbox.insert(0, "uwb_waypoints.yaml")
        self.select_path_button = tk.Button(self, text="Select",
                                            command=self.select_path)
        
        self.start_button = tk.Button(self, text="Start",
                                            command=self.start_wpf)
        
        self.stop_button = tk.Button(self, text="Stop",
                                            command=self.stop_wpf)
        self.stop_button.config(state=tk.DISABLED)
        
        self.load_label.grid(row=0, column=0, sticky='ew')
        self.load_textbox.grid(row=0, column=1, sticky='ew')
        self.select_path_button.grid(row=0, column=2, sticky='ew')
        self.start_button.grid(row=1, column=0, sticky='ew')
        self.stop_button.grid(row=1, column=2, sticky='ew')
        
        self.selected_wps_pub = self.create_publisher(
            PointStamped, "/igw_gps_points", 1)
        
        self.state_check_timer = self.create_timer(1, self.check_nav_state)
        self.state_check_timer.cancel()

        
    def select_path(self):
        self.load_textbox.delete(0, tk.END)
        self.load_textbox.insert(0, filedialog.askopenfilename())
    
    def uwb_waypoints_yaml_loader(self, uwb_wps_file_path: str):
        uwb_wps_list = None
        with open(uwb_wps_file_path, 'r') as file:
            uwb_wps_dict = yaml.safe_load(file)
            
            uwb_wps_list = []
            for wp in uwb_wps_dict['uwb_waypoints']:
                uwb_wps_list.append((wp['x'], wp['y'], wp['yaw']))
        
        return uwb_wps_list

    def start_wpf(self):
        self.uwb_wps = self.uwb_waypoints_yaml_loader(self.load_textbox.get())
        # transform to map frame
        transform = self.tf_buffer.lookup_transform(
            self.map_frame, self.uwb_ref_frame, rclpy.time.Time()
        )
        
        transformed_wps = []
        for wp in self.uwb_wps:
            posest_temp = PoseStamped()
            posest_temp.pose.position.x = wp[0]
            posest_temp.pose.position.y = wp[1]
            posest_temp.pose.position.z = 0.0
            
            #TODO: orientation
            # posest_temp.pose.orientation.x = 0.0
            # posest_temp.pose.orientation.y = 0.0
            # posest_temp.pose.orientation.z = 0.0
            # posest_temp.pose.orientation.w = 1.0
            
            
            target_rot = R.from_euler('z', float(wp[2]), degrees=False)
            target_quat = target_rot.as_quat()
            
            posest_temp.pose.orientation.x = target_quat[0]
            posest_temp.pose.orientation.y = target_quat[1]
            posest_temp.pose.orientation.z = target_quat[2]
            posest_temp.pose.orientation.w = target_quat[3]
            
            posest_temp.header.stamp = self.get_clock().now().to_msg()
            posest_temp.header.frame_id = ""
            
            transformed_wp = tf2_geometry_msgs.do_transform_pose_stamped(
                posest_temp, transform
            )
            transformed_wps.append(transformed_wp)
            self.get_logger().info(f"Transformed waypoint: {transformed_wp}")
            
            
        # Command to navigate to waypoints
        # self.navigator.waitUntilNav2Active(localizer='robot_localization')
        self.navigator.followWaypoints(transformed_wps)
        
        # Start the state check timer
        if self.state_check_timer.is_canceled():
            self.state_check_timer.reset()


    def stop_wpf(self):
        self.navigator.cancelTask()
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)

        # Stop the state check timer    
        if not self.state_check_timer.is_canceled():
            self.state_check_timer.cancel()
            
    def check_nav_state(self):
        if self.navigator.isTaskComplete():
            self.get_logger().info("WPS Navigation task completed")
            self.stop_wpf()
            


def main(args=None):
    rclpy.init(args=args)
    uwb_waypoints_follower_node = UwbWaypointsFollowerNode()
    
    while rclpy.ok():
        rclpy.spin_once(uwb_waypoints_follower_node, timeout_sec=0.1)
        uwb_waypoints_follower_node.update()