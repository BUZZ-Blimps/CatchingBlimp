# Welcome to the new ROS2 MicroROS integrated blimp version!



## MICRO ROS

To enable the MicroROS agent, please make the microros_ws bash, and do 'colcon build':

```
source .../microros_ws/install/setup.bash  #(recommend taking it out of the GitHub and make a copy, for the GitHub directory is shared by devices)
cd /microros_ws
colcon build
```
	
To make bash, it can be done by going into your bashrc:

```
nano ~/.bashrc
source dir/setup.bash
```
 
Alternatively, just go into the directory, and source the setup files:

```
. install/setup.bash
```
	
The MicroROS agent can be run using the ros2 run command:

```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0  
```

Note: ttyACM0 is the serial port name that teensy is connected to.

For more information, see the [Pi -> MICRO ROS](#micro-ros-1) section.



## ROS2

The ROS2 publisher is in the ROS_tf2_test folder, make sure the teensy is plugged into the pi for the tf2 messages to be broadcasted!
To enable the publisher, please build first:

```
cd /ROS_tf2_test
colcon build
```

Now, go into ROS_tf2_test and run the following: 

```
source $PWD/install/setup.bash
```
 
Alternatively, if VS Code is used, make sure the JSON configurator include path is the following:

```
"includePath": ["${workspaceFolder}/**","/opt/ros/foxy/include" ]
```	
Note: This should be set up already, but if it's not done, make sure to include the VS Code folder generated in the OG microros-teensy code and copy paste it.

To launch the tf (transformation) publisher, run the ros2 launch command:

```
ros2 launch blimp_telemetry blimp_telemetry.launch.py
```	
	
## Pi

### Ubuntu

To get Ubuntu on Rasberry Pi 4, follow the instructions here: https://ubuntu.com/tutorials/how-to-install-ubuntu-desktop-on-raspberry-pi-4#1-overview\
	
Note: Make sure the version is 20.04.5 LST (64-bit)

The Ubuntu server should now be on the pi. Here is how to log in:

- default username: ubuntu
- default password: ubuntu

After the first log in, the system will prompt a passward change, set it to: 

```
raspberrypi
```

To change to username of the pi, reboot first:

```
sudo reboot
```
 
Create a password for root:

```
sudo passwd root
```

Now, log in as the root user:
	
- username: root
- password: The password you set (i.e. raspberrypi)
 
Then change the username to pi for the ubuntu user:

```
usermod -l pi ubuntu
groupmod -n pi ubuntu
usermod -d /home/pi -m pi
```
 
After this, log out of root by doing -> [Ctrl] + [D].

Try logging back into it with the new username:

- username: pi
- password: rasberrypi
 
### WiFi Setup + SSH

To enable SSH, WiFi needs to be set up first:

Tutorial video -> https://www.youtube.com/watch?app=desktop&v=s4ZDlV3tIuM

- WiFi SSID: COREBlimp
- Password: jollypiano265
 
Now, get excited, you are ready to get SSH working!

Run the following:

```
sudo apt update
sudo apt upgrade
sudo apt install openssh-server
```
	
Now it's done!

Try on another computer to ssh into the Pi:

- ssh pi@192.168.0.10# where # is the Pi ID
- Enter your password

You now possess the power to get into the Pi wirelessly!

After getting into the Pi directory, we can install ROS 2 and MICRO ROS.

### ROS 2

Follow one of the options below for ROS 2 installation.

Option 1
- Open a terminal and copy the file InstallROS2.sh to the pi using the following command:

```
scp InstallROS2.sh pi@192.168.0.10#:/home/pi/
```

- Don't forget to source:

```
nano ~/.bashrc
source /opt/ros/foxy/setup.bash
source ~/.bashrc
```	

Option 2
- Follow the installation instructions here: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
- Don't forget to source:

```
nano ~/.bashrc
source /opt/ros/foxy/setup.bash
source ~/.bashrc
```

### MICRO ROS

Follow one of the options below for MICRO ROS installation.

Option 1
- Open a terminal and copy the folder microros_ws to the pi using the following command:

```
scp -r microros_ws pi@192.168.0.10#:/home/pi/
```

- Remove the build and install folder in microros_ws using the command:

```
rm -r [folder]
```
 
Option 2
- Follow the instructions here to setup MICRO ROS: https://manzurmurshid.medium.com/how-to-connect-teensy-3-2-with-micro-ros-and-ros2-foxy-6c8f99c9b66a
- Before "colcon build", make sure colcon is installed.
- Follow the instructions here to install colcon: https://colcon.readthedocs.io/en/released/user/installation.html
		
Run the following commands:

```
sudo sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add 
sudo apt update
sudo apt install python3-colcon-common-extensions
```

Potential Error:
- WARNING: when running "ros2 run micro_ros_setup build_agent.sh", the system may throw errors saying "pytest version too old"

- When this error happens, run the following:

```
pip3 install --upgrade pytest
```

- The system might tell you that you already have the newest version, that is a CAP, after running the command above the agent builds with no errors.

Now, run the following commands:

```
colcon build
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

After these steps, everything is set up for the Pi, to run the MICRO ROS agent, run the forementioned micro_ros_agent command:

```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 
```
  
Note: ttyACM0 is the serial port name that teensy is connected to.

Tip: To see what port that you are running on, plug a Teensy in your device and run the following:

```
ls -1 /dev > dev.txt
```

Unplug your Teensy, and run:

```
ls -1 /dev > dev2.txt
```

Finally run: 

```
diff dev.txt dev2.txt
```

The port name in /dev/tty will now be shown.

  
****Now you are ready to fly your blimp! Yay!****
	

  
  
  
  
  
  
  



