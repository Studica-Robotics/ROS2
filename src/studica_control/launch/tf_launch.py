"""
This launches manual composition AND the required transforms for SLAM and NAV2. 
Run the Lidar separately.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
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
    
    nodes = [
        manual_composition,
        base_tf,
        # laser_tf
    ]

    return LaunchDescription(nodes)