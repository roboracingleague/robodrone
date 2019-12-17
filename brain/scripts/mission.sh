#!/bin/bash
#To launch a mission from a file containing waypoints path
filename=$1
if [ "$filename" = "" ] 
then
    echo "Error argument"
    exit 1
fi
if [ ! -f $filename ] 
then
    echo "Error $filename not found"
    exit 1
fi

fichier=./tmp.mission
echo -e "" > $fichier
cat $filename >> $fichier

rostopic pub /robocar/mission setpoint_leader/robocars_mission "$(<$fichier)"