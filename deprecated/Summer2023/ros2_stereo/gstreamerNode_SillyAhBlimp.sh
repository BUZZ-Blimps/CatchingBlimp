export GSCAM_CONFIG="udpsrc address="192.168.0.200" port=3002 ! application/x-rtp, encoding-name=H264, payload=96 ! rtph264depay ! h264parse ! queue ! avdec_h264 ! videoconvert"

#ros2 run gscam2 gscam_main --ros-args --remap /image_raw:=/SillyAhBlimp/sync/image_raw -p namespace:=SillyAhBlimp
ros2 launch gscam2 node_param_launch.py camera_name:=SillyAhBlimp

read -r -d '' _ </dev/tty

