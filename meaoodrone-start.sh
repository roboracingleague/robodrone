#/bin/sh

nohup roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557" gcs_url:="udp://@127.0.0.1"&
sleep 10
nohup roslaunch realsense2_camera rs_t265.launch &
nohup rosrun setpoint_leader offb_fsm_raw_node &
