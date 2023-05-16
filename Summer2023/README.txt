Welcome to the new ROS2 MicroROS integrated blimp version!

MICRO ROS:
To enbale the MicroROS agent, please make the microros_ws bash, and do "colcon build"
	- source .../microros_ws/install/setup.bash
	- cd /microros_ws
	- colcon build
	
**To make bash, it can be done through going into bashrc
	-nano ~/.bashrc
	-source dir/setup.bash
  or, just go into the directory, and source the setup files:
	-. install/setup.bash
	
The MicroROS agent can be run using the ros2 run command
	-ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0  
	 ttyACM0 is the serial port name that teensy is connected to



ROS2:
The ROS2 publisher on pi is in the piROS_testfolder, make sure the 
To enable the publisher, please build 
	-cd /piROS_test
	-colcon build
	
	*If VS code is used, make sure the JSON configurator include path is as follows:
            -"includePath": [
             "${workspaceFolder}/**",
             "/opt/ros/foxy/include"

To launch the publisher,  run ros2 launch command    
	-ros2 launch blimp_telemetry blimp_telemetry.launch.py



