#!/bin/bash

# Check if the command and name arguments are provided
if [ "$#" -lt 4 ]; then
    echo "Usage: $0 <command> <name> <encoder_num> <params>"
    exit 1
fi

COMMAND=$1
NAME=$2
N_ENCODER=$3
PARAMS=${4:-''}  # Use the fourth argument if provided, otherwise set to an empty string

# Call the ROS 2 service with the provided arguments
# Modify values in initparams as needed
ros2 service call /manage_dynamic_publisher studica_control/srv/SetData "{command: '$COMMAND', name: '$NAME', component: 'titan', params: '$PARAMS', initparams: {n_encoder: $N_ENCODER, dist_per_tick: 0.0006830601, speed: 0.8, can_id: 45, motor_freq: 15600}}"
