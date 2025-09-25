from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('studica_control')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    # Joy node to read from the gamepad hardware
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    # Container to hold the gamepad component
    container = ComposableNodeContainer(
        name='gamepad_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='studica_control',
                plugin='studica_control::GamepadController',
                name='gamepad_controller',
                parameters=[params_file]
            )
        ],
        output='screen'
    )

    nodes = [joy_node, container]
    return LaunchDescription(nodes)
