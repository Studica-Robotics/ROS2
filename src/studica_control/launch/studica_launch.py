from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
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
        parameters=[params_file],
        remappings=[
            ('/cmd_vel', '/cmd_vel_smoothed'),
            ('cmd_vel', '/cmd_vel_smoothed'),
        ],
    )

    velocity_smoother = LifecycleNode(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        namespace='',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
            ('cmd_vel_smoothed', '/cmd_vel_smoothed'),
            ('odom', 'odom'),
        ],
    )

    velocity_smoother_lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='velocity_smoother_lifecycle_manager',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['velocity_smoother'],
        }],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml')]
    )

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

    rosbridge_server = ExecuteProcess(
        cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
        output='screen'
    )

    nodes = [velocity_smoother, velocity_smoother_lifecycle_mgr, manual_composition, ekf, base_tf, imu_tf, rosbridge_server]
    return LaunchDescription(nodes)