#!/bin/sh
v4l2-ctl -d /dev/video0 -c exposure_auto=1 -c exposure_auto_priority=0 -c exposure_absolute=157
~/opencv_workspace/detectBalloon_w_UART/build/detectBalloon_w_UART