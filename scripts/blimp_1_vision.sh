#!/bin/bash

source $CATCHING_BLIMP_PATH/Summer2023/ros2_stereo/install/setup.bash

ros2 launch stereo_combined blimp_vision.launch.xml blimp_name:=BurnCreamBlimp camera_id:=1