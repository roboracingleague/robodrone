# 1 Dev environment

## 1.0 Prerequisites :
Install mavros
Install librealsense and realsense-ros

## 1.1 Get source :

```sh
mkdir -p 
git clone thisrepo
```

## 2 Get tools

Follow http://dev.px4.io/en/setup/dev_env_mac.html

## 3 Compile

Create a catkin_env with src/setpoint_leader link to setpoint_leader
```sh
mkdir -p catkin_ws/src
cd catkin_ws/src
ln -s $ROBODRONE_DIR/setpoint_leader setpoint_leader
```
then compile 
```
cd ..
catkin_make
catkin config --init
source ./devel/setup.bash
```

## 4 Start 

Launch mavros master
- through PX4 IP
```sh
roslaunch mavros px4.launch fcu_url:="udp://:14540@[PX4_IP]:14557" gcs_url:="udp://@[GCS_IP]"
```
or through hardware interface connected to PX4 (ex here with USB on jetson nano)
```sh
roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0:921600" gcs_url:="udp://@[GCS_IP]"
```

Launch brain
```sh
rosrun setpoint_leader offb_fsm_raw_node
```

Launch t265 camera
```sh
roslaunch realsense2_camera rs_t265.launch
```


## 12 Monitor topic
### 1 Current rover position (reported by PX4 via mavlink)
```sh
rostopic echo mavros/local_position/pose
```

### 2 Current destination (sent by Ros module to PX4 via mavlink)
```sh
rostopic echo /mavros/setpoint_raw/local
```


## 14.1 Send mission manually :

```sh
rostopic pub /robocar/mission setpoint_leader/robocars_mission  "
departure:
  frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat:  150.0
  y_long: 300.0
  z_alt: 0.0
arrival:
  frame: 4
  command: 16
  is_current: true
  autocontinue: true
  param1: 0.0
  param2: 0.0
  param3: 0.0
  param4: 0.0
  x_lat:  0.0
  y_long: 150.0
  z_alt: 0.0
path:
    - frame: 4
      command: 16
      is_current: true
      autocontinue: true
      param1: 0.0
      param2: 0.0
      param3: 0.0
      param4: 0.0
      x_lat:  150.0
      y_long: 300.0
      z_alt: 0.0
    - frame: 4
      command: 16
      is_current: true
      autocontinue: true
      param1: 0.0
      param2: 0.0
      param3: 0.0
      param4: 0.0
      x_lat:  100.0
      y_long: 250.0
      z_alt: 0.0
    - frame: 4
      command: 16
      is_current: true
      autocontinue: true
      param1: 0.0
      param2: 0.0
      param3: 0.0
      param4: 0.0
      x_lat:  50.0
      y_long: 200.0
      z_alt: 0.0
    - frame: 4
      command: 16
      is_current: true
      autocontinue: true
      param1: 0.0
      param2: 0.0
      param3: 0.0
      param4: 0.0
      x_lat:  0.0
      y_long: 150.0
      z_alt: 0.0"
```
or from a file : 
```sh
./script/mission.sh filename
```

## 14.2 Mission recording
To record a mission from the drone, you need to configure your RC (associate channels to buttons) as following :
- channel 10 : for mode switching between onRecordMission mode and onWaitMission mode 
  - Without this channel active, default mode is onWaitMission
  - switching to onRecordMission activates channel 11 listening and start a new mission recording. When you switch back to onWaitMission mode, the mission recorded is saved in a file named <span style="color:red">**mission-recorded_*timestamp*.mission**</span> in the directory where you launch the brain
- channel 11 : to record a position, switch this channel

## 14.3 Brain State diagram
![State Diag](doc/statdiag.png)


## 16 change thresh to detect checkpoint/
```sh
rosparam set dst_thresh 0.2
```

## 19 Debug topic
```sh
rostopic echo robocar/debug
```

## 20 Wifi on Jetson Nano in Headless Mode
```sh
sudo nmcli dev wifi connect (NETWORK) password (PASSWORD) ifname wlan0
```

## 21 RC setup (taranis X9D-Plus)
Arming : SE (channel 12)
- disarmed
- armed
- armed and log activated (pose, velocity, local_raw_pub, attitude_raw_pub) 

Flight mode : SA (channel 5)
- Position
- Stabilized
- Accro

Brain mode : SB (channel 10)
- record mission
- wait mission
- execute last mission recorded

Record waypoint : SF (channel 11)

Emergency disarming : SD (channel 8)

Record waypoint mode : SC (channel 6) - TODO
- normal
- freestyle 1
- freestyle 2

Switch offBoard/FlightMode : SH (channel 7)
- in offboard mode, switch to manual mode : if done during a mission, the mission is paused
- and vice versa (and mission resumes)

Pause during mission : SG (channel 9)
- pause back
- pause 1 : loiter
- pause 2 : land and disarm