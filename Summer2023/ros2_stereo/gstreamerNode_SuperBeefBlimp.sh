export GSCAM_CONFIG="udpsrc port=3006 ! application/x-rtp, encoding-name=H264, payload=96 ! rtph264depay ! h264parse ! queue ! avdec_h264 ! videoconvert"

#ros2 run gscam2 gscam_main --ros-args --remap /image_raw:=/SillyAhBlimp/sync/image_raw -p namespace:=SuperBeefBlimp
ros2 launch gscam2 node_param_launch.py camera_name:=SuperBeefBlimp

read -r -d '' _ </dev/tty
