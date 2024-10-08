#!/bin/bash

# Toggle digital output on and off every 0.5 seconds
while true; do
    # Set the pin high
    ros2 service call /manage_dynamic_publisher studica_control/srv/SetData "{command: 'cmd', name: 'q', component: 'dio',  params: 'set_hi'}" &
    sleep 0.3

    # Set the pin low
    ros2 service call /manage_dynamic_publisher studica_control/srv/SetData "{command: 'cmd', name: 'q', component: 'dio',  params: 'set_lo'}" &
   sleep 0.3
done

