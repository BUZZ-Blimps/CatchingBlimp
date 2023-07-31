# ROS2 Image pipeline for creating a point cloud

### 1. Publish raw image:
   
```	
ros2 run opencv_telemetry minimal_opencv_ros2_node 0
```

### 2. Split the images to left and right frames, and upload the camera info for both lenses:

```
ros2 run opencv_telemetry split_sync_images
```

### 3. Rectify each image by calling the launch file:

```
ros2 launch image_proc image_proc.launch.py
```

### 4. Create disparity for the image:

```
ros2 run stereo_image_proc disparity_node
```

To view the image, run:

``` 
ros2 run image_view disparity_view --ros-args --remap image:=disparity
```

To dynamically tune the disparity parameters:

```
ros2 run rqt_reconfigure rqt_reconfigure
```

### 5. Create point clound:

```
ros2 run stereo_image_proc point_cloud_node 
```

To view the point cloud in rviz, run:

```
ros2 run rviz2 rviz2
```
	
**IMPORTANT: The global frame MUST BE SET TO "BurnCreamBlimp_left_optical_frame"**
For creating the reference frame:

```
ros2 run tf2_ros static_transform_publisher   0.64 0 0.004  0 0 1.5708   BurnCreamBlimp_left_optical_frame   BurnCreamBlimp_left_optical_frame_child
```	


***!!!REMEMBER to rebuild the package by deleting the "build install log" folders and using "colcon build" in the "ros2_stereo" directory to reconfigure the setup.bash!!!***

***TO DO***

1. Create another node to subcribe to the ML custome messages and the point cloud messages to find the positions of targets (in the camera reference frame).
2. Tune disparity parameters (use rqt_reconfigure).

Tutorial reference:
- For calibration: http://wiki.ros.org/camera_calibration?distro=noetic
- For the entire process: https://jeffzzq.medium.com/ros2-image-pipeline-tutorial-3b18903e7329


plz read the documentations, thank you:)
