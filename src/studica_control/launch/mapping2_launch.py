"""
Autonomous mapping launch script using LiDAR and SLAM.
The robot will generate a map as you drive it around. Use: ros2 run teleop_twist_keyboard teleop_twist_keyboard
To save the map, open a new, privledged terminal and run: ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap
"""
# studica_launch.py is no longer included and must be launched separately.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('studica_control')
    
    # Declare launch arguments
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.04',
        description='SLAM map resolution in meters per pixel (e.g., 0.02 for 2cm cells, 0.05 for 5cm cells)'
    )
    
    # Hardcoded parameter file paths
    lidar_1_file = os.path.join(pkg_share, 'config', 'ydlidar_f.yaml')
    lidar_2_file = os.path.join(pkg_share, 'config', 'ydlidar_r.yaml')
    mapper_params_file = os.path.join(os.path.dirname(pkg_share), '..', '..', '..', 'nav2_params', 'mapper_params_online_sync.yaml')
    merger_params_file = os.path.join(pkg_share, 'config', 'merger_params.yaml')

    # Publish base_footprint -> base_link as a managed Node (cleaner shutdown than ExecuteProcess)
    base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    )
    
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_imu',
        arguments=['0.0924043', '-0.061896', '0.1511', '0', '0', '0', '1', 'base_link', 'imu_link']
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

    # Run slam_toolbox with all parameters from YAML file
    slam = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            mapper_params_file,
            {'resolution': LaunchConfiguration('resolution')}  # Override resolution from launch arg
        ]
    )

    # Launch Orbbec depth camera (DISABLED - USB issues)
    # camera = ExecuteProcess(
    #     cmd=['ros2', 'launch', 'orbbec_camera', 'gemini_e.launch.py'],
    #     output='screen'
    # )

    # Convert depth camera pointcloud to laserscan
    pointcloud_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/merged_cloud'),  # Merger's PointCloud2 output
            ('scan', '/merged_scan')        # SLAM subscribes to this
        ],
        parameters=[mapper_params_file]
    )

    # Merge LiDAR scans: /scan1 + /scan2 -> /merged_scan
    merger = Node(
        package='ros2_laser_scan_merger',
        executable='ros2_laser_scan_merger',
        name='ros2_laser_scan_merger',
        output='screen',
        parameters=[merger_params_file]
    )

    # Static transform for merged laser frame
    tf3 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_laser_merged',
        arguments=['0', '0', '0.02', '0', '0', '0', '1', 'base_link', 'laser']
    )

    # Foxglove Bridge for visualization
    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen'
    )

    nodes = [
        resolution_arg,  # Include the launch argument
        base_tf,
        imu_tf,
    #   laser_tf,
        tf1,
        lidar1,
        tf2,
        lidar2,
        # camera,              # DISABLED - USB issues
        pointcloud_to_scan,    # Convert merged_cloud to scan
        merger,                # Merge 2 LiDAR scans
        tf3,
        slam,
        foxglove,
    ]

    return LaunchDescription(nodes)
