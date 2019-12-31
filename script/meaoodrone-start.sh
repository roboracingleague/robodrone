#!/bin/bash
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
SCRIPT_DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )"

sleep 20
source $SCRIPT_DIR/init.sh


#start ros master
stdbuf -oL roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0:921600" gcs_url:="udp://@$GCSIP" >$LOG_DIR/rosmaster.log 2>&1 &
sleep 7

#set ros params
rosparam set lap_numbers 1
rosparam set mission_recorded_filename $MISSION_RECORDED_DIR/mission-recorded.mission
rosparam set mission_script $ROBODRONE_DIR/brain/scripts/mission.sh

#start camera ros node
stdbuf -oL roslaunch realsense2_camera rs_t265.launch >$LOG_DIR/ts_t265.log 2>&1 &

#start brain
stdbuf -oL rosrun setpoint_leader offb_fsm_raw_node >$LOG_DIR/setpoint_leader.log 2>&1 &
exit 0