from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('studica_control')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    # Run the standard joystick driver node provided by the joy package
    gamepad_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([gamepad_node])
