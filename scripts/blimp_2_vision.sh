#!/bin/bash

source $CATCHING_BLIMP_PATH/Summer2023/ros2_stereo/install/setup.bash
source /home/corelab/Luke_ML_Blimp/ros2_tracks_ws/install/setup.bash

ros2 launch stereo_combined blimp_vision.launch.xml blimp_name:=SillyAhBlimp camera_id:=2