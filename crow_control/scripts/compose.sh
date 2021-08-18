#!/bin/bash
while getopts ":hvnedg" opt; do
  case $opt in
    h)
      echo "Possible options:"
      echo -e "-h \t Displays this help (you already probably new that)."
      echo ""
      echo "= Pipeline selection ="
      echo -e "-v \t Run vision only pipeline (cameras, detector, locator, filter, server, adder)."
      echo -e "-e \t Run edge detector (instead of normal yolact detector)."
      echo -e "-n \t Do NOT run NLP pipeline."
      echo -e "-a \t Do NOT run PCL aggregation pipelines (pcl_cacher, pcl_aggregator)."
      echo ""
      echo "= Debug & Visualization ="
      echo -e "-g \t Run graphical visualization."
      echo -e "-d \t Debug mode (runs some debugging nodes)."
      echo ""
      echo "Warning! There is no sanity check, use of options is error checked by your own logic."
      exit 0
      ;;
    v)
      echo "Selected vision only pipeline." >&1
      O_VISION_ONLY=true
      ;;
    n)
      echo "Will not run the NLP pipeline." >&1
      O_NO_NLP=true
      ;;
    e)
      echo "Will use Yolact Edge." >&1
      O_EDGE=true
      ;;
    a)
      echo "Will not run pcl aggregation nodes." >&1
      O_NO_PCL_AGG=true
      ;;
    g)
      echo "Will start visualization node." >&1
      O_VIZ=true
      ;;
    d)
      echo "Will start debugging node(s)." >&1
      O_DEBUG=true
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

CMD="vision onto"

if [ "$O_VISION_ONLY" != true ]; then
    CMD="${CMD} control"
    if [ "$O_NO_NLP" != true ]; then
        CMD="${CMD} nlp"
    fi
fi
if [ "$O_NO_PCL_AGG" != true ]; then
    CMD="${CMD} pcl_aggregate"
fi
if [ "$O_EDGE" == true ]; then
    CMD="${CMD} detector_edge"
else
    CMD="${CMD} detector"
fi

if [ "$O_VIZ" == true ]; then
    CMD="${CMD} vizualization"
fi
if [ "$O_DEBUG" == true ]; then
    CMD="${CMD} debug"
fi


jcompose "${BASH_SOURCE%/*}/ros_compose.yaml" --jobs ${CMD}