#/bin/sh

LOG_DIR=$HOME/log
if [ ! -d $LOG_DIR ]
then
    mkdir $LOG_DIR
fi
MISSION_RECORDED_DIR=$HOME/mission_recorded
if [ ! -d $MISSION_RECORDED_DIR ]
then
    mkdir $MISSION_RECORDED_DIR
fi

export GCSIP=$(arp | grep 00:50:b6:a1:76:a8 | awk '{print $1}')

stdbuf -oL roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0:921600" gcs_url:="udp://@$GCSIP" >$LOG_DIR/rosmaster.log 2>&1 &
sleep 5
stdbuf -oL roslaunch realsense2_camera rs_t265.launch >$LOG_DIR/ts_t265.log 2>&1 &
stdbuf -oL rosrun setpoint_leader offb_fsm_raw_node >$LOG_DIR/setpoint_leader.log 2>&1 &

#set ros params
rosparam set lap_numbers 1
rosparam set mission_recorded_filename $HOME/mission_recorded/mission-recorded.mission
rosparam set mission_script $HOME/brain/scripts/mission.sh
