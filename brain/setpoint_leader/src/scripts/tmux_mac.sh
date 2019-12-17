tmux new -s dro -d

tmux split-window -h -t dro
tmux split-window -h -t dro
#tmux split-window -h -t dro
#tmux select-layout -t dro tiled
#tmux split-window -v -t dro
#tmux select-layout -t dro tiled
#tmux split-window -v -t dro
#tmux select-layout -t dro tiled
#tmux split-window -v -t dro
#tmux select-layout -t dro tiled
#tmux split-window -v -t dro
#tmux select-layout -t dro tiled
#tmux split-window -v -t dro
#tmux select-layout -t dro tiled

#tmux select-layout -t dro tiled

# C-m permet de gerer le retour chariot
# tmux send-keys -t dro:0.0 'date' C-m

# mavros
tmux send-keys -t dro:0.0 'source /home/obee/catkin_mavros/devel/setup.bash;roslaunch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600 gcs_url:=udp://@192.168.1.72'

# transformations realsense
# tmux send-keys -t dro:0.1 'rosrun tf2_ros static_transform_publisher 0 0 0 -1.57 0 0 camera_odom_frame local_origin'

# camera T265
tmux send-keys -t dro:0.1 'source /home/obee/catkin_ws/devel/setup.bash;roslaunch realsense2_camera rs_t265.launch'

# brain
tmux send-keys -t dro:0.2 'rosrun setpoint_leader offb_fsm_raw_node'

# tmux send-keys -t dro:0.4 'rostopic pub robocar/customer std_msgs/String -1 go'

# tmux send-keys -t dro:0.5 'rosnode kill setpoint_leader'

# tmux send-keys -t dro:0.6 'rosrun mavros mavsafety disarm'

# tmux send-keys -t dro:0.7 'rostopic echo /mavros/local_position/pose'

# lancer la mission dans waypoints.md
