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

nohup roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557" gcs_url:="udp://@127.0.0.1" 1>$LOG_DIR/rosmaster.log 2>&1 &
sleep 5
#nohup roslaunch realsense2_camera rs_t265.launch 1>$LOG_DIR/ts_t265.log 2>&1 &
nohup rosrun setpoint_leader offb_fsm_raw_node 1>$LOG_DIR/setpoint_leader.log 2>&1 &

#set ros params
rosparam set lap_numbers 3
rosparam set mission_recorded_filename $HOME/mission_recorded/mission-recorded.mission
rosparam set mission_script $HOME/meaoodrone/robodrone/brain/scripts/mission.sh
