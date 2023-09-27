# Calibration Tutorial

1. calibration example with 8x6 checker board, 25 mm square width:

on linux terminal, run:

```
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.12 right:=/BurnCreamBlimp/right/image_raw left:=/BurnCreamBlimp/left/image_raw left_camera:=/BurnCreamBlimp/left right_camera:=/BurnCreamBlimp/right
```

3. move the checker board around for sampling at every angle, click "calibrate" after it lights up. Confirm the calibration on the interface, click "save", the file will be saved in the ***tmp*** directory on ***Computer***

4. Rename the ymal file camera names to "elp_left" and "elp_right". Move the two yaml files into the opencv_ros2 workspace:
    ->install->opencv_telemetry->share->opencv_telemetry->calibration   >>>make the calibration folder:) 


***TO DO***

1. calibrate all cameras and find a way to check the quality of the calibration files
2. create the calibration files for each camera




tutorial reference: http://wiki.ros.org/camera_calibration?distro=noetic
