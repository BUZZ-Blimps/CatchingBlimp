export GSCAM_CONFIG="udpsrc port=3001 ! application/x-rtp, encoding-name=H264, payload=96 ! rtph264depay ! h264parse ! queue ! avdec_h264 ! videoconvert"

#ros2 run gscam2 gscam_main --ros-args --remap /image_raw:=/BurnCreamBlimp/sync/image_raw -p gscam_publisher:=burn_cream_blimp_gscam_publisher
ros2 launch gscam2 node_param_launch.py camera_name:=BurnCreamBlimp 

#read -r -d '' _ </dev/tty

