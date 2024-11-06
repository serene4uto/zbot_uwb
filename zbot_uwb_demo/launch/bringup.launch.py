from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os





def generate_launch_description():
    
    demo_params_dir = os.path.join(get_package_share_directory('zbot_uwb_demo'), 'param')
    sensor_params_file = os.path.join(demo_params_dir, 'sensor_params.yaml')
    rl_ekf_params_file = os.path.join(demo_params_dir, 'rl_ekf_params.yaml')
    
    mw_md_params = os.path.join(get_package_share_directory('zbot_stella_n2_robot'),
                                    'params', 'mw_md.yaml')
    
    print(sensor_params_file)
    
    declare_use_imu = DeclareLaunchArgument(
        'use_imu',
        default_value='false',
        description='Whether to use the imu or not'
    )

    declare_use_lidar = DeclareLaunchArgument(
        'use_lidar',
        default_value='false',
        description='Whether to use the lidar or not'
    )
    
    declare_use_rl = DeclareLaunchArgument(
        'use_rl',
        default_value='false',
        description='Whether to use the robot_localization or not'
    )

    use_lidar = LaunchConfiguration('use_lidar')
    use_imu = LaunchConfiguration('use_imu')
    use_rl = LaunchConfiguration('use_rl')

    zbot_stella_n2_bringup_group_action = GroupAction([

        # Description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('zbot_stella_n2_description'), 'launch', 'description.launch.py']
            ))
        ),

        # Base Bringup
        Node(
            package='zbot_stella_n2_robot_base',
            executable='base_node',
            name='base_node',
            output='screen',
            parameters=[mw_md_params],
            remappings=[
                ('cmd_vel', 'zbot_stella_n2_velocity_controller/cmd_vel_unstamped'),
                ('odom', 'mw_md/odom'),
            ]
        ),

        # Base Teleop
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('zbot_stella_n2_control'), 'launch', 'teleop_base.launch.py']
            ))
        ),

        # Joy Teleop
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('zbot_stella_n2_control'), 'launch', 'teleop_joy.launch.py']
            ))
        ),

    ])

    zbot_stella_n2_sensors_group_action = GroupAction([

        # IMU
        Node(
            package='umx_driver',
            executable='um7_driver',
            name='um7_driver',
            parameters=[sensor_params_file],
            output='screen',
            condition=IfCondition(use_imu),
            remappings=[
                ('imu/data', 'zb_imu/data'),
                ('imu/mag', 'zb_imu/mag'),
                ('imu/yaw', 'zb_imu/yaw'),
            ]
        ),

        #IMU Filter
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_node',
            output='screen',
            parameters=[sensor_params_file],
            condition=IfCondition(use_imu),
            remappings=[
                ('imu/data_raw', 'zb_imu/data'),
                ('imu/mag', 'zb_imu/mag')
            ]
        ),

        Node(package='ydlidar',
            executable='ydlidar_node',
            name='ydlidar_node',
            output='screen',
            emulate_tty=True,
            parameters=[sensor_params_file],
            namespace='/',
            remappings=[
                ('scan', 'front/scan')
            ],
            condition=IfCondition(use_lidar)
        )
    ])
        
    
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[rl_ekf_params_file],
        condition=IfCondition(use_rl)
    )
    

    return LaunchDescription([
        declare_use_imu,
        declare_use_lidar,
        declare_use_rl,
        zbot_stella_n2_bringup_group_action,
        zbot_stella_n2_sensors_group_action,
        robot_localization_node
    ])