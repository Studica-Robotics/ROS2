from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os

def generate_launch_description():

    pkg_prefix = get_package_prefix('studica_control')
    pkg_share  = get_package_share_directory('studica_control')

    exec_path      = os.path.join(pkg_prefix, 'lib', 'studica_control', 'manual_composition')
    default_params = os.path.join(pkg_share, 'config', 'params.yaml')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to params.yaml. Override to use a config from your own project: '
                    'params_file:=/path/to/your/params.yaml',
    )

    # Run as root so pigpio can access /dev/mem and lock its PID file.
    # The sudoers rule installed by scripts/setup_permissions.sh allows this
    # without a password prompt.
    manual_composition = ExecuteProcess(
        cmd=[
            'sudo', '-E', exec_path,
            '--ros-args',
            '-r', '__node:=control_server',
            '--params-file', LaunchConfiguration('params_file'),
        ],
        output='screen',
    )

    return LaunchDescription([params_arg, manual_composition])
