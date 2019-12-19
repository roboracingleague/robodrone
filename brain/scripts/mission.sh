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

#
nb_lap=1
if [ $# -gt 1 ]
then
    nb_lap=$2
fi

fichier=./tmp.mission

if [ $nb_lap -le 1 ]
then
    echo -e "" > $fichier
    cat $filename >> $fichier
else
    #TOTEST
    #if nb > 1, loop and repeat path (except last frame) nb_lap times
    nb_line_head=$(grep -n "path:" $filename  | awk -F":" '{print $1}')
    last_frame_num=$(grep -n "frame:" $filename  | awk -F":" '{print $1}' | tail -1)
    nb_line=$(wc -l $filename | awk '{print $1}')

    echo -e "" > $fichier
    head -$nb_line_head $filename >> $fichier
    i=0
    while [ $i -lt $nb_lap ]
    do
        head -$(($last_frame_num-1)) $filename | tail -$(($last_frame_num-$nb_line_head-1)) >> $fichier
        i=$(($i+1))
    done
    tail -$(($nb_line-$last_frame_num+1)) $filename >> $fichier

fi
rostopic pub /robocar/mission setpoint_leader/robocars_mission "$(<$fichier)" &
exit 0