from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    declared_arguments = []
    
    # Your robot control node
    manual_composition = Node(
        package='studica_control',
        executable='manual_composition',
        name='manual_composition',
        output='screen'
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
        laser_tf
    ]

    return LaunchDescription(declared_arguments + nodes)