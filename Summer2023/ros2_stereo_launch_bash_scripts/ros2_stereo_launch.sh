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
