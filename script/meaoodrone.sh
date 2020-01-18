#!/bin/bash
rosmaster_start () {
        echo "starting ros_master..."
        stdbuf -oL roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0:921600" gcs_url:="udp://@$GCSIP" >>$LOG_DIR/rosmaster.log 2>&1 &
}

rosmaster_stop () {
        echo "stopping ros_master..."
        PID=$(ps -ef | grep "roslaunch mavros px4.launch fcu_url" | grep -v grep | awk '{print $2}')
        [[ "$PID" != "" ]] && kill $PID
}

t265_start () {
        echo "starting t265 ros node..."
        stdbuf -oL roslaunch realsense2_camera rs_t265.launch >>$LOG_DIR/ts_t265.log 2>&1 &
}

t265_stop () {
        echo "stopping t265..."
        PID=$(ps -ef | grep "roslaunch realsense2_camera rs_t265.launch" | grep -v grep | awk '{print $2}')
        [[ "$PID" != "" ]] && kill $PID
}

brain_start () {
        echo "starting t265 ros node..."
        stdbuf -oL rosrun setpoint_leader offb_fsm_raw_node >>$LOG_DIR/setpoint_leader.log 2>&1 &
}

brain_stop () {
        echo "stopping brain..."
        PID=$(ps -ef | grep "rosrun setpoint_leader offb_fsm_raw_node" | grep -v grep | awk '{print $2}')
        [[ "$PID" != "" ]] && kill $PID
}

SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
SCRIPT_DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )"

action=""
[[ "$1" == "stop" ]] && action="stop"
[[ "$1" == "start" ]] && action="start"

if [[ "$action" == "start" ]]
then

        sleep 20
        source $SCRIPT_DIR/init.sh


        #start ros master
        rosmaster_start
        sleep 7

        #set ros params
        rosparam set lap_numbers 1
        rosparam set mission_recorded_filename $MISSION_RECORDED_DIR/mission-recorded.mission
        rosparam set mission_script $ROBODRONE_DIR/brain/scripts/mission.sh

        #start camera ros node
        t265_start

        #start brain
        brain_start
fi
if [[ "$action" == "stop" ]]
then
        source $SCRIPT_DIR/init.sh
        t265_stop
        brain_stop
        rosmaster_stop
fi
