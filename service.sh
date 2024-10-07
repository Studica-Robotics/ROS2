#!/bin/bash

# Check if the command and name arguments are provided
if [ "$#" -lt 3 ]; then
    echo "Usage: $0 <command> <name> <component> [params]"
    exit 1
fi

COMMAND=$1
NAME=$2
COMPONENT=$3
PARAMS=${4:-''}  # Use the fourth argument if provided, otherwise set to an empty string

# Call the ROS 2 service with the provided arguments
ros2 service call /manage_dynamic_publisher studica_control/srv/SetData "{command: '$COMMAND', name: '$NAME', component: '$COMPONENT', params: '$PARAMS'}"
