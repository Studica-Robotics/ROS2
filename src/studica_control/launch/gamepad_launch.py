from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('studica_control')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    # Run the external gamepad_controller_node provided by the joy package
    gamepad_node = Node(
        package='joy',
        executable='gamepad_controller_node',
        name='gamepad_controller',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([gamepad_node])
