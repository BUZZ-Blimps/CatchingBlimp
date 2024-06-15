#!/bin/bash

cd ml_model/weights
cp best.pt best_test_720p_4.pt
cp best_test_720p_4.pt /home/corelab/Luke_ML_Blimp/ros2_tracks_ws/src/Yolov5andDeepSort_ROS2_Node/track_ros2/track_ros2/weights
cd /home/corelab/Luke_ML_Blimp/ros2_tracks_ws/src/Yolov5andDeepSort_ROS2_Node/track_ros2/track_ros2/weights

