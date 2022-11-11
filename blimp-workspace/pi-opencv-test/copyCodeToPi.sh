#!/bin/bash
if [ "$1" != "" ]; then
	hostname=$1
	
	#Check if device is online (2 second timeout)
	wget -q --tries=1 --timeout=2 -O - $hostname > /dev/null
	if [ $? == 0 ]; then
		echo ">>Copying workspace source code files to pi@$hostname"
		ssh pi@$hostname 'mkdir -p /home/pi/pi-opencv-test/build'
		
		scp -r ../src/* pi@$hostname:/home/pi/pi-opencv-test/
		
		#echo "Killing program."
		#ssh pi@$hostname 'sudo killall -9 piStereoVision'
		
		echo "Compiling code."
		ssh pi@$hostname 'make -C /home/pi/pi-opencv-test'
		
		echo ">>Done."
	else
		echo "$hostname is offline :("
	fi
else
	pwd
	echo "Usage: copyCodeToPi.sh [hostname]"
fi
