#!/bin/bash
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
SCRIPT_DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )"

export ROBODRONE_DIR=$SCRIPT_DIR/..

export LOG_DIR=$ROBODRONE_DIR/log
if [ ! -d $LOG_DIR ]
then
    mkdir $LOG_DIR
fi

export MISSION_RECORDED_DIR=$ROBODRONE_DIR/mission_recorded
if [ ! -d $MISSION_RECORDED_DIR ]
then
    mkdir $MISSION_RECORDED_DIR
fi

source /opt/ros/melodic/setup.bash
source $HOME/catkin_ws/devel/setup.bash
source $HOME/catkin_ws2/devel/setup.bash

#prerequesite : in your /home/dlinano/.bashmeaoo, set your GCS_MacAdress
source /home/dlinano/.bashmeaoo
export GCSIP=$(arp | grep $GCS_MacAdress | awk '{print $1}')

alias rrun="roslaunch mavros px4.launch fcu_url:=\"/dev/ttyACM0:921600\" gcs_url:=\"udp://@$GCSIP\""
alias brun='rosrun setpoint_leader offb_fsm_raw_node'
alias rl='rostopic list'
function re() { rostopic echo $1; }
