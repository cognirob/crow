#!/bin/bash

function crow () {
    # conda deactivate
    cd ~/crow2
    source install/setup.bash
}

function runcmd() {
    sleep 1; xdotool type --delay 1 --clearmodifiers "crow && $@"; xdotool key Return;
}

function new_tab() {
    xdotool windowfocus "$1"
    xdotool key ctrl+shift+t
}

WID=$(xprop -root | grep "_NET_ACTIVE_WINDOW(WINDOW)"| awk '{print $5}')

runcmd "ros2 launch crow_vision_ros2 full_crow_object.launch.py"
new_tab $WID

runcmd "ros2 run crow_vision_ros2 filter"
new_tab $WID

runcmd "ros2 run crow_ontology server"
new_tab $WID

runcmd "echo Waiting for Ontology server to come online"
sleep 5
runcmd "ros2 run crow_ontology adder"
new_tab $WID

runcmd "ros2 run crow_control ovis"
xdotool windowfocus $WID
xdotool key ctrl+188
new_tab $WID

runcmd "ros2 run crow_nlp sentence_processor"
new_tab $WID

runcmd "ros2 run crow_nlp nl_mic"

wmctrl -i -a $WID
