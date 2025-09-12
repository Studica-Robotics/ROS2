from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():

    pkg_share = get_package_share_directory('studica_control')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')
 
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)

    # The manual_composition node is a container for components.
    # We need to pass the parameters for each component to the container.
    # The container will then pass them to the correct component upon loading.
    # The parameters in the YAML are nested under component names, 
    # so we load the whole file as parameters for the container node.
    manual_composition = Node(
        package='studica_control',
        executable='manual_composition',
        name='control_server',
        output='screen',
        parameters=[params]
    )

    nodes = [
        manual_composition
    ]

    return LaunchDescription(nodes)
