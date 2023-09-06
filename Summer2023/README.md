# :tada: Welcome to the new ROS2 MicroROS integrated Blimp V8! :tada:



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

*For More info about ROS2, please refer to the official website of ROS2 (This is required for all software team to do):* https://docs.ros.org/en/foxy/index.html

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
"includePath":
["${workspaceFolder}/**",
"/opt/ros/foxy/include" ]
```	
Note: This should be set up already, but if it's not done, make sure to include the VS Code folder generated in the OG microros-teensy code and copy paste it.

To launch the tf (transformation) publisher, run the ros2 launch command:

```
ros2 launch blimp_telemetry blimp_telemetry.launch.py
```	
	
## Pi

### Ubuntu

To get Ubuntu on Rasberry Pi 4, follow the instructions here: https://ubuntu.com/tutorials/how-to-install-ubuntu-desktop-on-raspberry-pi-4#1-overview\
(We are using Orange Pi 5 now, but the process is similar)
	
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

### ORANGE PI 5B
On Steve Downloads there is a ubunutu 22.04.2 file. Flash that OS to a SD card and then plug the SD card into the orange pi.
When setting up the login ideally use a numbering system like "orangepi1", "orangepi2", ... etc.
Set the password to 1234.

Next connect to the COREBlimp wifi.

You can assign a static IP through the router and then restart the orange pi if you want.

Next, SCP the microros_ws, turnEyesOn.sh, openDocker.sh, and the docker steps .md file from an old orange pi to a new one.
Then we need to setup docker. Run the following commands:
```
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg
echo \
  "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

So we don't need to sudo to use docker:

```
sudo groupadd docker
sudo gpasswd -a $USER docker
```

Now install the foxy docker image 

```
sudo docker pull ros:foxy-ros-base-focal
```
Now edit the openDocker.sh file and change the the file path at /home/orangepi# to whatever you named this orangepi you are setting up.
The openDocker.sh file should now work without a problem and you should reference the readme file for where to navigate to use the micro_ros startup.

The turnEyesOn.sh should work without any problems.

Just start both of these scripts via an ssh terminal.
You can use the "screen" command to run multiple bash scripts in one terminal if you want

### ROS 2

Follow one of the options below for ROS 2 installation.

Option 1
- Open a terminal and copy the file InstallROS2.sh to the pi using the following command:

```
scp InstallROS2.sh pi@192.168.0.10#:/home/pi/
```
- To run InstallROS2.sh:

```
./InstallROS2.sh
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

- Remember to rebuild the package (read *option 2* for how to install colcon package):

```
colcon build
```
 
Option 2
- Follow the instructions here to setup MICRO ROS: https://manzurmurshid.medium.com/how-to-connect-teensy-3-2-with-micro-ros-and-ros2-foxy-6c8f99c9b66a
- **Before "colcon build", make sure colcon is installed**.
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

This will show the port name for /dev/tty.

  
****Now you are ready to fly your blimp! Yay!****

## Flying the Blimp

Follow the instructions for flying a blimp.

1. Attach battery/batteries to the pcb with Orange pi and teensy connected, while the camera is also connected to the Orange pi (Make sure the battery checkers are plugged in, and they should say approximately 8.4V)
2. Make sure that the base station is running and it's on "COREBlimp" Wifi
3. ssh into the pi in **TWO** separate shells (# is the number of the pi) (password for all the pis should be 1234):
```
 ssh orangepi#@192.168.0.11#
```
4. In the first shell, run:
```
./openDocker.sh
```
This will start the docker, and then:
```
./ros_entrypoint.sh 
cd /home/ 
source install/setup.bash 
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```
The blimp will be activated, and it will self correct its yaw. You should check the polarity of the motors at this point to make sure the motors are plugged in correctly.

To connect/switch to a blimp, click on the down bottom on the key pad on the X-box controller.

5. In another shell (not in the two that's already in the orangepi), run:
```
./runCamera1.sh 
```
and in another shell, run (first cd into the bash script directory):
```
cd ros2_stereo_launch_bash_scripts/
./ros2_stereo_launch.sh
```
and in another shell(XD), run:
```
./runMLNode.sh
```
This will start the stream line of the vision code from machine learning to point cloud.

6. In the second orange pi shell, run:  
```
./turnEyesOn.sh
```
this will start the UDP dump of the video feed, and the blimp is good to fly both manually and atonomously!

Some commands on the X-box controller to be remembered:

joysticks: three axis motions

RT: auto/manual switch

RB: open/close gate switch

LB: shoot/stop fan+gate switch


**TO DO**
1. Combine every bash script into one for each blimp.
  
  
  
  
  



