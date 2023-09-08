# ros2_robot_model

**To install the URDF exporter in Solidworks, follow this tutorial:**

https://www.youtube.com/watch?v=I08lO_SRBbk

URDF exporter download link:

https://github.com/ros/solidworks_urdf_exporter/releases
(choose the newest release v1.6.1, only the .exe file is required)

In Solidworks, the exporter can be added by accessing **Options**->**Add-ins**, then in the ***Other Add-ins*** block click the box with ***"SW2URDF"***

What would Billy Mays say? But wait, there's more!

**insert awkward laugh*

After the URDF file is exported:


**To make the package for the URDF robot model, please follow this after using the SOLIDWORKS built-in sw2urdf tool:**

https://github.com/SWAMP-Blimps/CatchingBlimp/tree/main/Summer2023/ros2_sw2urdf


To launch the robot model:
```
ros2 launch test_urdf_tool launch.py 
```
Important: before launching this robot model, make sure the tf message is being broadcasted (see the launch file in the tf package)


[tf_2 package](https://github.com/SWAMP-Blimps/CatchingBlimp/tree/main/Summer2023/ROS_tf2_test)

***TO DO:***
1. Make a more accurate model of the blimp and make it the robot model for the launch file.
2. Change the package name to ros2_robot_model
   
