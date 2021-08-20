#!/bin/bash

# O_VISION_ONLY=0
# O_NO_VISION=0
# O_NO_NLP=0
# O_NO_LOGIC=0
# O_VIZ=0
# O_DEBUG=0
# O_NO_SPLIT=0

while getopts ":hvonlgds" opt; do
  case $opt in
    h)
      echo "Possible options:"
      echo -e "-h \t Displays this help (you already probably new that)."
      echo ""
      echo "= Pipeline selection ="
      echo -e "-v \t Run vision only pipeline (cameras, detector, locator, filter, server, adder)."
      echo -e "-o \t Run everything but the vision pipeline (i.e. no cameras, detector, locator, filter, adder)."
      echo -e "-n \t Do not run NLP pipeline."
      echo -e "-l \t Omit logic node. Can be combined with other pipeline options."
      echo ""
      echo "= Debug & Visualization ="
      echo -e "-g \t Run graphical visualization."
      echo -e "-d \t Debug mode (runs some debugging nodes)."
      echo ""
      echo "= Run modes ="
      echo -e "-s \t Do not split the terminal."
      echo ""
      echo "Warning! There is no sanity check, use of options is error checked by your own logic."
      exit 0
      ;;
    v)
      echo "Selected vision only pipeline." >&1
      O_VISION_ONLY=true
      ;;
    o)
      echo "Will not run the vision pipeline." >&1
      O_NO_VISION=true
      ;;
    n)
      echo "Will not run the NLP pipeline." >&1
      O_NO_NLP=true
      ;;
    l)
      echo "Will not start the logic control (or dummy robot, if debug was selected)." >&1
      O_NO_LOGIC=true
      ;;
    g)
      echo "Will start visualization node." >&1
      O_VIZ=true
      ;;
    d)
      echo "Will start debugging node(s)." >&1
      O_DEBUG=true
      ;;
    s)
      echo "Will not perform terminal window split." >&1
      O_NO_SPLIT=true
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
    :)
      echo "Option -$OPTARG requires an argument." >&2
      exit 1
      ;;
  esac
done

function crow () {
    # conda deactivate
    cd ~/crow2
    source install/setup.bash
}

function set_title() {
  if [[ -z "$ORIG" ]]; then
    ORIG=$PS1
  fi
  TITLE="\[\e]2;$*\a\]"
  PS1=${ORIG}${TITLE}
}

function runcmd() {
    sleep 1; xdotool type --delay 1 --clearmodifiers "crow && $@"; xdotool key Return;
}

function new_tab() {
    xdotool windowfocus "$1"
    xdotool key ctrl+shift+t
}

WID=$(xprop -root | grep "_NET_ACTIVE_WINDOW(WINDOW)"| awk '{print $5}')

if [ "$O_NO_VISION" != true ]; then
    # stty -echo
    runcmd "echo -ne '\033]30;Cameras\007'"
    # stty -echo
    runcmd "ros2 launch crow_vision_ros2 all_cameras.launch.py"

    new_tab $WID
    sleep 15
    runcmd "echo -ne '\033]30;Detection\007'"
    runcmd "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/imitrob/TensorRT-7.2.1.6/lib"
    runcmd "ros2 launch crow_vision_ros2 crow_object.launch.py"

    new_tab $WID
    runcmd "echo -ne '\033]30;Filter\007'"
    runcmd "ros2 run crow_vision_ros2 filter"

    new_tab $WID
    runcmd "echo -ne '\033]30;Cacher\007'"
    runcmd "ros2 run crow_vision_ros2 pcl_cacher"

    new_tab $WID
    runcmd "echo -ne '\033]30;Marker\007'"
    runcmd "ros2 run crow_vision_ros2 marker_detector"
fi

new_tab $WID
runcmd "echo -ne '\033]30;Server\007'"
runcmd "ros2 run crow_ontology server"

if [ "$O_VIZ" = true ]; then
    new_tab $WID
    runcmd "echo -ne '\033]30;GViz\007'"
    runcmd "ros2 run crow_control wxvis"
fi

new_tab $WID
runcmd "echo -ne '\033]30;OViz\007'"
runcmd "echo Waiting for Ontology server to come online"
runcmd "ros2 service call /ontology_server/get_parameters rcl_interfaces/srv/GetParameters names:\ ['database_host']\ "
runcmd "ros2 run crow_ontology ovis"

if [ "$O_NO_SPLIT" != true ]; then
    xdotool windowfocus $WID
    xdotool key ctrl+188
fi

if [ "$O_NO_VISION" != true ]; then
    new_tab $WID
    runcmd "echo -ne '\033]30;Adder\007'"
    runcmd "ros2 run crow_ontology adder"
fi

if [ "$O_VISION_ONLY" = true ]; then
    echo "Vision pipeline is running, exiting."
    exit 0
fi

if [ "$O_NO_LOGIC" != true ]; then
    if [ "$O_DEBUG" = true ]; then
        new_tab $WID
        runcmd "echo -ne '\033]30;Dummy robot\007'"
        runcmd "ros2 run crow_control dummy"
    fi
    new_tab $WID
    runcmd "echo -ne '\033]30;Logic\007'"
    runcmd "ros2 run crow_control logic"
fi
if [ "$O_NO_SPLIT" != true ]; then
    xdotool windowfocus $WID
    xdotool key ctrl+188
fi

if [ "$O_NO_NLP" != true ]; then
    new_tab $WID
    runcmd "echo -ne '\033]30;NL Processor\007'"
    runcmd "ros2 run crow_nlp sentence_processor"

    new_tab $WID
    runcmd "echo -ne '\033]30;NL Input\007'"
    runcmd "ros2 run crow_nlp nl_snow --lang cs --ends 0.5 --gain 0.5"
fi

wmctrl -i -a $WID
