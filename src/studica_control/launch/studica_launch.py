from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('studica_control')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    # Nodes from launch_nodes.py

    rosbridge_server = ExecuteProcess(
        cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
        output='screen'
    )

    orbbec_camera = ExecuteProcess(
        cmd=['ros2', 'launch', 'orbbec_camera', 'gemini_e.launch.py'],
        output='screen'
    )

    manual_composition = Node(
        package='studica_control',
        executable='manual_composition',
        name='control_server',
        output='screen',
        parameters=[params_file]
    )

    # Service calls from launch_nodes.py
    # These are delayed to run after the manual_composition node starts
    service_calls = [
        RegisterEventHandler(
            OnProcessStart(
                target_action=manual_composition,
                on_start=[
                    ExecuteProcess(
                        cmd=['ros2', 'service', 'call', '/create_component', 'studica_control/srv/SetData',
                             '"{component: \'titan\', initparams: {can_id: 42, motor_freq: 15600, dist_per_tick: 0.0006830601, speed: 0.8}}"'],
                        shell=True
                    ),
                    ExecuteProcess(
                        cmd=['ros2', 'service', 'call', '/create_component', 'studica_control/srv/SetData',
                             '"{name: \'servo1\', component: \'servo\', initparams: {pin: 13, servo_type: \'standard\'}}"'],
                        shell=True
                    ),
                    ExecuteProcess(
                        cmd=['ros2', 'service', 'call', '/create_component', 'studica_control/srv/SetData',
                             '"{name: \'servo2\', component: \'servo\', initparams: {pin: 12, servo_type: \'standard\'}}"'],
                        shell=True
                    ),
                    ExecuteProcess(
                        cmd=['ros2', 'service', 'call', '/create_component', 'studica_control/srv/SetData',
                             '"{name: \'servo3\', component: \'servo\', initparams: {pin: 14, servo_type: \'standard\'}}"'],
                        shell=True
                    ),
                    ExecuteProcess(
                        cmd=['ros2', 'service', 'call', '/create_component', 'studica_control/srv/SetData',
                             '"{name: \'servo4\', component: \'servo\', initparams: {pin: 15, servo_type: \'standard\'}}"'],
                        shell=True
                    )
                ]
            )
        )
    ]

    ld = LaunchDescription()

    ld.add_action(rosbridge_server)
    ld.add_action(orbbec_camera)
    ld.add_action(manual_composition)

    for call in service_calls:
        ld.add_action(call)

    return ld
