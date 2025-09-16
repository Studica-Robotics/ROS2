"""
Autonomous mapping launch script using LiDAR and SLAM.
The robot will generate a map as you drive it around. Use: ros2 run teleop_twist_keyboard teleop_twist_keyboard
To save the map, open a new, privledged terminal and run: ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap
"""

from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


import os
import time
import yaml

def generate_launch_description():
    pkg_share = get_package_share_directory('studica_control')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')
    lidar_1_file = LaunchConfiguration('params_file1')
    lidar_2_file = LaunchConfiguration('params_file2')
    params1_arg = DeclareLaunchArgument(
        'params_file1',
        default_value=os.path.join(pkg_share, 'config', 'ydlidar_f.yaml'),
        description='Parameters file for YDLidar #1')
    params2_arg = DeclareLaunchArgument(
        'params_file2',
        default_value=os.path.join(pkg_share, 'config', 'ydlidar_r.yaml'),
        description='Parameters file for YDLidar #2')

    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen'
    )

    ros_bridge = ExecuteProcess(
        cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml']
    )

    merge = ExecuteProcess(
        cmd=['ros2', 'launch', 'ros2_laser_scan_merger', 'merge_2_scan.launch.py'],
        output='screen'
    )
 
    manual_composition = Node(
        package='studica_control',
        executable='manual_composition',
        name='control_server',
        output='screen',
        parameters=[params_file]
    )

    base_tf = ExecuteProcess(
        cmd=[[
            'ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_link'
        ]],
        shell=True
    )
    
    laser_tf = ExecuteProcess(
        cmd=[[
            'ros2 run tf2_ros static_transform_publisher --x 0.144 --y 0 --z 0 --qx 0 --qy 0 --qz 0.7071 --qw -0.7071 --frame-id base_link --child-frame-id laser_frame'
        ]],
        shell=True

    )

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


    nodes = [
        params1_arg,
        params2_arg,
        LogInfo(msg=['Using YDLidar #1 params: ', LaunchConfiguration('params_file1')]),
        LogInfo(msg=['Using YDLidar #2 params: ', LaunchConfiguration('params_file2')]),
        ros_bridge,
        manual_composition,
        base_tf,
        # laser_tf,
        tf1,
        lidar1,
        tf2,
        lidar2,
        merge
    ]

    return LaunchDescription(nodes)
