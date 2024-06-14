#!/bin/bash
if [ "$1" != "" ]; then
	hostname=$1
	
	#Check if device is online (1 second timeout)
	wget -q --tries=1 --timeout=1 -O - $hostname > /dev/null
	if [ $? == 0 ]; then
		echo ">>Copying workspace source code files to pi@$hostname"
		ssh pi@$hostname 'mkdir -p /home/pi/udp_client/build'
		
		scp -r ../src/* pi@$hostname:/home/pi/udp_client/
		
		#echo "Killing program."
		#ssh pi@$hostname 'sudo killall -9 piStereoVision'
		
		echo "Compiling code."
		ssh pi@$hostname 'make -C /home/pi/udp_client'
		
		echo ">>Done."
	else
		echo "$hostname is offline :("
	fi
else
	pwd
	echo "Usage: copyCodeToPi.sh [hostname]"
fi
