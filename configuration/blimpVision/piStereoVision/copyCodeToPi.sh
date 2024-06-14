#!/bin/bash
if [ "$1" != "" ]; then
	hostname=$1
	
	echo ">>Removing old workspace from pi@$hostname"
	ssh pi@$hostname 'rm -rf /home/pi/piStereoVisionSrc'
	
	echo ">>Copying workspace source code files to pi@$hostname"
	scp -r ./piStereoVisionSrc pi@$hostname:/home/pi/
	
	echo "Killing program."
	ssh pi@$hostname 'sudo killall -9 piStereoVision'
	
	echo "Compiling code."
	ssh pi@$hostname 'make -C /home/pi/piStereoVisionSrc'
	
	echo ">>Done."
else
	echo "Usage: copyCodeToPi.sh [hostname]"
fi

