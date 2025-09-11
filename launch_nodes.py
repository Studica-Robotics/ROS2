import subprocess

def launch_ros2_nodes_and_services():

    environment_commands = """
    cd /home/vmx/ROS2/
    . /opt/ros/humble/setup.bash
    . install/setup.bash
    export LD_LIBRARY_PATH=/usr/local/lib/studica_drivers:$LD_LIBRARY_PATH
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/studica_control
    """

    #Nodes
    joy_node_command = f"{environment_commands}\nros2 run joy game_controller_node"
    rosbridge_command = f"{environment_commands}\nros2 launch rosbridge_server rosbridge_websocket_launch.xml"
    orbbec_camera_command = f"{environment_commands}\nros2 launch orbbec_camera gemini_e.launch.py"
    slam_toolbox_command = f"{environment_commands}\nros2 launch slam_toolbox online_sync_launch.py"
    navigation_command = f"{environment_commands}\nros2 launch nav2_bringup navigation_launch.py"
    manual_composition_command = f"{environment_commands}\nros2 run studica_control manual_composition"

    #srvs calls
    service_calls = [
        f"{environment_commands}\nros2 service call /create_component studica_control/srv/SetData \"{{component: 'titan', initparams: {{can_id: 42, motor_freq: 15600, dist_per_tick: 0.0006830601, speed: 0.8}}}}\"",
        f"{environment_commands}\nros2 service call /create_component studica_control/srv/SetData \"{{name: 'servo1', component: 'servo', initparams: {{pin: 13, servo_type: 'standard'}}}}\"",
        f"{environment_commands}\nros2 service call /create_component studica_control/srv/SetData \"{{name: 'servo2', component: 'servo', initparams: {{pin: 12, servo_type: 'standard'}}}}\"",
        f"{environment_commands}\nros2 service call /create_component studica_control/srv/SetData \"{{name: 'servo3', component: 'servo', initparams: {{pin: 14, servo_type: 'standard'}}}}\"",
        f"{environment_commands}\nros2 service call /create_component studica_control/srv/SetData \"{{name: 'servo4', component: 'servo', initparams: {{pin: 15, servo_type: 'standard'}}}}\"",
    ]

    try:
        #Launch nodes
        joy_node_process = subprocess.Popen(
            ["bash", "-c", joy_node_command],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        print("joy_node launched successfully.")

        rosbridge_process = subprocess.Popen(
            ["bash", "-c", rosbridge_command],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        print("rosbridge_server node launched successfully.")

        orbbec_camera_process = subprocess.Popen(
            ["bash", "-c", orbbec_camera_command],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        print("orbbec_camera node launched successfully.")

        slam_toolbox_process = subprocess.Popen(
            ["bash", "-c", slam_toolbox_command],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        print("slam_toolbox node launched successfully.")

        navigation_process = subprocess.Popen(
            ["bash", "-c", navigation_command],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        print("navigation node launched successfully.")

        manual_composition_process = subprocess.Popen(
            ["bash", "-c", manual_composition_command],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        print("manual_composition node launched successfully.")

        print("Launching ROS2 service calls...")

        
        for service_call in service_calls:
            service_process = subprocess.run(
                ["bash", "-c", service_call],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            print("STDOUT:", service_process.stdout.decode())
            print("STDERR:", service_process.stderr.decode())
            if service_process.returncode != 0:
                print(f"Service call failed: {service_call}")
                break
        else:
            print("All service calls executed successfully.")

        print("The script will remain running to keep all nodes alive.")

        #Wait for all node processes to terminate manually
        joy_node_process.wait()
        rosbridge_process.wait()
        orbbec_camera_process.wait()
        slam_toolbox_process.wait()
        navigation_process.wait()
        manual_composition_process.wait()

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    launch_ros2_nodes_and_services()
