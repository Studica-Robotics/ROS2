"""
Autonomous mapping launch script using LiDAR and SLAM.
The robot will generate a map as you drive it around. Use: ros2 run teleop_twist_keyboard teleop_twist_keyboard
To save the map, open a new, privledged terminal and run: ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap
"""
# studica_launch.py is no longer included and must be launched separately.

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('studica_control')
    lidar_1_file = LaunchConfiguration('params_file1')
    lidar_2_file = LaunchConfiguration('params_file2')
    use_merger = LaunchConfiguration('use_merger')
    scan_topic = LaunchConfiguration('scan_topic')
    params1_arg = DeclareLaunchArgument(
        'params_file1',
        default_value=os.path.join(pkg_share, 'config', 'ydlidar_f.yaml'),
        description='Parameters file for YDLidar #1')
    params2_arg = DeclareLaunchArgument(
        'params_file2',
        default_value=os.path.join(pkg_share, 'config', 'ydlidar_r.yaml'),
        description='Parameters file for YDLidar #2')
    use_merger_arg = DeclareLaunchArgument(
        'use_merger',
        default_value='true',
        description='Start ros2_laser_scan_merger and publish /merged_scan from /scan1 + /scan2 + depth camera')
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/merged_scan',
        description='LaserScan topic for slam_toolbox to subscribe to (e.g., /merged_scan, /scan1, or /scan2)')
    
    # Path to SLAM toolbox mapper params
    mapper_params_file = os.path.join(os.path.dirname(pkg_share), '..', '..', '..', 'nav2_params', 'mapper_params_online_sync.yaml')

    # Publish base_footprint -> base_link as a managed Node (cleaner shutdown than ExecuteProcess)
    base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    )
    
    #laser_tf = ExecuteProcess(
    #    cmd=[[
    #       'ros2 run tf2_ros static_transform_publisher --x 0.144 --y 0 --z 0 --qx 0 --qy 0 --qz 0.7071 --qw -0.7071 --frame-id base_link --child-frame-id laser_frame'
    #    ]],
    #    shell=True
    #)

    lidar1 = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar1',
        emulate_tty=True,
        output='screen',
        parameters=[lidar_1_file],
        remappings=[('scan', '/scan1')],
        namespace='/'
    )

    tf1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_laser1',
        arguments=['0.144', '0', '0.02', '0', '0', '0', '1', 'base_link', 'laser1_frame']
    )

    lidar2 = LifecycleNode(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar2',
            output='screen',
            emulate_tty=True,
            parameters=[lidar_2_file],
            remappings=[('scan', '/scan2')],
            namespace='',
    )

    tf2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_laser2',
        arguments=['-0.144', '0', '0.02', '0', '0', '1', '0', 'base_link', 'laser2_frame']
    )

    # Run slam_toolbox directly so we can set scan_topic via parameters
    slam = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            mapper_params_file,
            { 'scan_topic': scan_topic }
        ]
    )

    # Launch Orbbec depth camera (DISABLED - USB issues)
    # camera = ExecuteProcess(
    #     cmd=['ros2', 'launch', 'orbbec_camera', 'gemini_e.launch.py'],
    #     output='screen'
    # )

    # Convert depth camera pointcloud to laserscan (DISABLED - camera disabled)
    pointcloud_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/base/custom_cloud'),
            ('scan', '/merged_scan')  # Avoid conflict with merger output
        ],
        parameters=[{
            'target_frame': 'laser',
            'transform_tolerance': 0.01,
            'min_height': -0.5,
            'max_height': 2.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.05,
            'range_max': 12.0,
            'use_inf': True,
        }],
        condition=IfCondition(use_merger)
    )

    # Merge LiDAR scans only: /scan1 + /scan2 -> /merged_scan
    merger_params_file = os.path.join(pkg_share, 'config', 'merger_params.yaml')
    merger = Node(
        package='ros2_laser_scan_merger',
        executable='ros2_laser_scan_merger',
        name='ros2_laser_scan_merger',
        output='screen',
        parameters=[
            merger_params_file,
            {
                'laserscan_topics': '/scan1 /scan2',
                'scan_destination_topic': '/merged_scan',
                'destination_frame': 'laser'
            }
        ],
        condition=IfCondition(use_merger)
    )

    tf3 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_laser_merged',
        arguments=['0', '0', '0.02', '0', '0', '0', '1', 'base_link', 'laser'],
        condition=IfCondition(use_merger)
    )


    nodes = [
        params1_arg,
        params2_arg,
        use_merger_arg,
        scan_topic_arg,
        LogInfo(msg=['Using YDLidar #1 params: ', LaunchConfiguration('params_file1')]),
        LogInfo(msg=['Using YDLidar #2 params: ', LaunchConfiguration('params_file2')]),
        base_tf,
    #   laser_tf,
        tf1,
        lidar1,
        tf2,
        lidar2,
        # camera,              # DISABLED - USB issues
        pointcloud_to_scan,  # DISABLED - camera disabled
        merger,                # Merge 2 LiDAR scans
        tf3,
        slam,
    ]

    return LaunchDescription(nodes)
