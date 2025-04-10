"""
Autonomous mapping launch script using LiDAR and SLAM.
The robot will generate a map as you drive it around. Use: ros2 run teleop_twist_keyboard teleop_twist_keyboard
To save the map, open a new, privledged terminal and run: ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess

import os
import time
import yaml

def generate_launch_description():

    pkg_share = get_package_share_directory('studica_control')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')
 
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
            'ros2 run tf2_ros static_transform_publisher 0 0 0 3.14159 0 0 base_link laser_frame'
        ]],
        shell=True
    )

    lidar = ExecuteProcess(
        cmd=['ros2', 'launch', 'ydlidar_ros2_driver', 'ydlidar_launch.py'],
        output='screen'
    )

    slam = ExecuteProcess(
        cmd=['ros2', 'launch', 'slam_toolbox', 'online_sync_launch.py'],
        output='screen'
    )

    nodes = [
        manual_composition,
        base_tf,
        laser_tf,
        lidar,
        slam,
    ]

    return LaunchDescription(nodes)
