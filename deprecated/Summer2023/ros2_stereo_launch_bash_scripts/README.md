ROS2 Image pipeline for creating a point cloud:
1. publish raw image:
	ros2 run opencv_telemetry minimal_opencv_ros2_node 0
2. split the images to left and right frames, and upload the camera info for both lenses:
	ros2 run opencv_telemetry split_sync_images 
3. rectify each image by calling the launch file:
	ros2 launch image_proc image_proc.launch.py 
4. create disparity for the image:
	ros2 run stereo_image_proc disparity_node
	--to view the image, run:
	ros2 run image_view disparity_view image:=disparity
5. create point clound:
	ros2 run stereo_image_proc point_cloud_node 

To view the point cloud in rviz, run:
	ros2 run rviz2 rviz2
	
IMPORTANT:--The global frame MUST BE SET TO "BurnCreamBlimp_left_optical_frame"--
for creating the reference frame:
	ros2 run tf2_ros static_transform_publisher   0.64 0 0.004  0 0 1.5708   BurnCreamBlimp_left_optical_frame   BurnCreamBlimp_left_optical_frame_child
	

tutorial reference:
for calibration: http://wiki.ros.org/camera_calibration?distro=noetic
https://jeffzzq.medium.com/ros2-image-pipeline-tutorial-3b18903e7329

