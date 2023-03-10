#!/bin/bash
if [ "$1" != "" ]; then
	hostname=$1
	
	# Check if device is online
	timeout=2
	ping $hostname -c 1 -W $timeout > /dev/null
	if [ $? == 0 ]; then
		echo ">>Copying workspace source code files to pi@$hostname"
		ssh pi@$hostname 'mkdir -p /home/pi/piOffboardStereoVision/build'
		
		scp -r ../src/* pi@$hostname:/home/pi/piOffboardStereoVision/
		
		#echo "Killing program."
		#ssh pi@$hostname 'sudo killall -9 piStereoVision'
		
		echo "Compiling code."
		ssh pi@$hostname 'make -C /home/pi/piOffboardStereoVision'
		
		echo ">>Done."
	else
		echo "$hostname is offline :("
	fi
else
	echo "Usage: copyCodeToPi.sh [hostname]"
fi
