#!/bin/bash
## Script to publish rcin channels to mavros/rc/in topic

#Edit the value you want to change and execute this script 
Flight_Mode="1000"
Brain_Mode="1500"
RecordWaypoint="1000"
SwitchMode="1000"
PauseMission="1000"

rostopic pub mavros/rc/in mavros_msgs/RCIn "1500 1500 1500 1500 $Flight_Mode 1500 $SwitchMode 1500 $PauseMission $Brain_Mode $RecordWaypoint 1500 1500 1500 1500 1500"