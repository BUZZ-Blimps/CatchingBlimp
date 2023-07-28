# Welcome to the new ROS2 MicroROS integrated blimp version!



## MICRO ROS

To enable the MicroROS agent, please make the microros_ws bash, and do "colcon build":

```
source .../microros_ws/install/setup.bash  #(recommend taking it out of the GitHub and make a copy, for the GitHub directory is shared by devices)
cd /microros_ws
colcon build
```
	
To make bash, it can be done through going into bashrc:

```
nano ~/.bashrc
source dir/setup.bash
```
 
Or, just go into the directory, and source the setup files:

```
. install/setup.bash
```
	
The MicroROS agent can be run using the ros2 run command:

```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0  
ttyACM0 is the serial port name that teensy is connected to
```

(more info available under the Pi section)
For more information, see the [Pi](#Pi) section.


## ROS2

The ROS2 publisher is in the ROS_tf2_test folder, make sure the teensy is plugged into the pi for the tf2 messages to be broadcasted!
To enable the publisher, please build first:

```
cd /ROS_tf2_test
colcon build
```

Note: Don't forget to make bash in bashrc: source .../install/setup.bash to get access to the whole directory name, go into ROS_tf2_test, and run: pwd (this replaces the ...)
 
Or, if VS Code is used, make sure the JSON configurator include path is as the following:

```
"includePath": ["${workspaceFolder}/**","/opt/ros/foxy/include" ]
```
	
(this should be set up already, but if it's not done, make sure to include the VS Code folder generated in the OG microros-teensy code and copy paste it)

To launch the tf (transformation) publisher, run the ros2 launch command:

```
ros2 launch blimp_telemetry blimp_telemetry.launch.py
```	
	
## Pi

### Ubuntu

To get Ubuntu on Rasberry Pi 4, follow the instructions on
 	-- https://ubuntu.com/tutorials/how-to-install-ubuntu-desktop-on-raspberry-pi-4#1-overview\
 	-- make sure the version is 20.04.5 LST (64-bit)
This will get Ubuntu server on the pi

The default ubuntu user name to log in to is: ubuntu, the passward is also: ubuntu
after the first log in, the system will prompt a passward change, set it to: raspberrypi

to change to username of the pi, reboot first:

	sudo reboot
 
create a password for root:

	sudo passwd root
log in root:
	username: root
	password: the password you set to(raspberrypi)
 
Then change the username to pi for the ubuntu user:

	-- usermod -l pi ubuntu
	-- groupmod -n pi ubuntu
	-- usermod -d /home/pi -m pi
 
after this, log out of root by doing -> [Ctrl] - [D]
try logging back into it with the new username:
	-- username: pi
	-- password: rasberrypi
 
### WiFi Setup + SSH

To enable SSH, WIFI needs to be set up first:
	*tutorial video-> https://www.youtube.com/watch?app=desktop&v=s4ZDlV3tIuM
	-WIFI SSID: COREBlimp
	-password: jollypiano265
 
now, get excited, you are ready to get SSH working!
Run the following:

	sudo apt update
	sudo apt upgrade
	sudo apt install openssh-server
	
now it's done!
try on another computer to ssh into the pi:
	--ssh pi@192.168.0.10# where # is pi ID
	--enter password
you now possess the power to get into the pi wirelessly!
after getting into the pi directory, we can install ROS2 and MICRO ROS	

### ROS 2

Follow one of the options below for ROS 2 installation.

	Option 1:
	-- Open a terminal and copy the file InstallROS2.sh to the pi using the following command:
	-- scp InstallROS2.sh pi@192.168.0.10#:/home/pi/
	-- don't forget to source:
 
	nano ~/.bashrc
	source /opt/ros/foxy/setup.bash
	run source ~/.bashrc
	
	Option 2:
	-- https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
	-- don't forget to source:
 
	nano ~/.bashrc
	source /opt/ros/foxy/setup.bash
	run source ~/.bashrc

### MICRO ROS

Follow the instructions below for MICRO ROS 2 installation.

Complete one of the following options:

	Option 1:
 	-- Open a terminal and copy the folder microros_ws to the pi using the following command:
 	-- scp -r microros_ws pi@192.168.0.10#:/home/pi/
 	-- Remove the build and install folder in microros_ws using the command rm -r [folder]

 
  	Option 2:
 	-- Follow the instructions on the website below:
 	-- https://manzurmurshid.medium.com/how-to-connect-teensy-3-2-with-micro-ros-and-ros2-foxy-6c8f99c9b66a
 	-- Before "colcon build", make sure colcon is installed
	  - Installing colcon:
		-reference:https://colcon.readthedocs.io/en/released/user/installation.html
		
  	Run the following commands:
  
	sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add 
	sudo apt update
	sudo apt install python3-colcon-common-extensions
	
--WARNING: when running "ros2 run micro_ros_setup build_agent.sh", the system may throw errors saying "pytest version too old",
when this happens, run this:
 -- pip3 install --upgrade pytest
the system might tell you that you already have the newest version, that is a CAP, after running this the agent builds with no errors
-- after "colcon build", and "ros2 run micro_ros_setup build_agent.sh" make sure to "source install/local_setup.bash" to make bash!
	
 After these steps, everything is set up for the pi, to run the MICRO ROS agent, run the forementioned micro_ros_agent command:
 
 	ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 
  
ttyACM0 is the serial port name that teensy is connected to
-tip: to see what port that you are running on, plug teensy in your device first, and run:
	 
  ls -1 /dev > dev.txt
--unplug your teensy, and run:
   
	ls -1 /dev > dev2.txt
   
--then run: 
	 	--diff dev.txt dev2.txt
the port name in /dev/tty will be shown

  
****Now you are ready to fly your blimp! Yay!****
	

  
  
  
  
  
  
  



