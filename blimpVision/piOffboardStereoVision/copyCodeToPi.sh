#!/bin/bash
if [ "$1" != "" ]; then
	hostname=$1
	
	# Check if device is online
	timeout=2
	ping $hostname -c 1 -W $timeout > /dev/null
	if [ $? == 0 ]; then
		echo ">>Removing old workspace source code files from pi@$hostname"
		ssh pi@$hostname 'rm -r /home/pi/piOffboardStereoVisionSrc/'
		
		echo ">>Copying workspace source code files to pi@$hostname"
		ssh pi@$hostname 'mkdir -p /home/pi/piOffboardStereoVisionSrc/build'
		scp -r ./src/* pi@$hostname:/home/pi/piOffboardStereoVisionSrc/
		
		echo "Compiling code."
		ssh pi@$hostname 'make -C /home/pi/piOffboardStereoVisionSrc'
		
		echo ">>Done."
	else
		echo "$hostname is offline :("
	fi
else
	echo "Usage: copyCodeToPi.sh [hostname]"
fi
