from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
import os
import launch.actions


def generate_launch_description():
    zbot_uwb_localization_dir = get_package_share_directory(
        "zbot_uwb_localization")
    rl_params_file = os.path.join(
        zbot_uwb_localization_dir, "rl_dual_ekf_uwb_params.yaml")

    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[rl_params_file, {"use_sim_time": False}],
                remappings=[("odometry/filtered", "odometry/local")],
            ),
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                output="screen",
                parameters=[rl_params_file, {"use_sim_time": False}],
                remappings=[("odometry/filtered", "odometry/global")],
            ),
            launch_ros.actions.Node(
                package="zbot_uwb_localization",
                executable="navuwb_transform_node",
                name="navuwb_transform",
                output="screen",
                parameters=[rl_params_file, {"use_sim_time": False}],
                remappings=[
                    ("imu/data", "imu/data"),
                    ("odometry/uwb", "odometry/uwb"),
                    ("odometry/filtered", "odometry/global"),
                ],
            ),
        ]
    )