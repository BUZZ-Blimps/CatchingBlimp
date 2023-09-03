#!/bin/bash

source /home/corelab/ros2_stereo/install/setup.bash

cd scripts

./split_sync.sh &
sleep 0.1

./rectify.sh &
sleep 0.1

./disparity.sh &
sleep 0.1

./point_cloud.sh
var1=$(pgrep -P $$)

read -r -d '' _ </dev/tty
#for var2 in $var1
#do
#  :
#  var3=$(pgrep -P $var2)
#  kill -9 $var3
#done

#./point_cloud.sh

