export GSCAM_CONFIG="udpsrc port=3000 ! application/x-rtp, encoding-name=H264, payload=96 ! rtph264depay ! h264parse ! queue ! avdec_h264 ! videoconvert"

ros2 run gscam2 gscam_main --ros-args --remap /image_raw:=/BurnCreamBlimp/sync/image_raw -p camera_info_url:=file://$PWD/src/gscam2/cfg/my_camera.ini 
