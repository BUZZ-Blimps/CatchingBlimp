#!/bin/bash
if [ "$1" != "" ]; then
	hostname=$1
	echo "Killing program."
	ssh pi@$hostname 'sudo killall -9 piOffboardStereoVision'
	
	echo "Deleting program."
	ssh pi@$hostname "rm -r /home/pi/piOffboardStereoVisionSrc/"
else
	echo "Usage: deleteProgram.sh [hostname]"
fi

