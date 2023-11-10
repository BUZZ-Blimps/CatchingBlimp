#!/bin/bash

. /opt/ros/foxy/setup.bash
. ~/GitHub/CatchingBlimp/Summer2023/ros2_stereo/install/setup.bash
. ~/Luke_ML_Blimp/ros2_tracks_ws/install/setup.bash
export GSCAM_CONFIG="udpsrc port=3005 ! application/x-rtp, encoding-name=H264, payload=96 ! rtph264depay ! h264parse ! queue ! avdec_h264 ! videoconvert"

ros2 launch launchVision.launch.xml
