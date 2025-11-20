from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

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

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml')]
    )

    rosbridge_server = ExecuteProcess(
        cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
        output='screen'
    )

    nodes = [manual_composition, ekf, rosbridge_server]
    return LaunchDescription(nodes)