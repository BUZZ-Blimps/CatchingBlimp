#!/bin/bash
if [ "$1" != "" ]; then
	hostname=$1
	echo "Killing program."
	ssh pi@$hostname 'sudo killall -9 piStereoVision'
	echo "Waiting 6 seconds before restarting..."
	sleep 6
	echo "Starting program."
	if [ "$2" == "" ]; then
		ssh pi@$hostname "./piStereoVisionSrc/build/piStereoVision"
	else
		ssh pi@$hostname "./piStereoVisionSrc/build/piStereoVision $2"
	fi
else
	echo "Usage: restartProgram.sh [hostname] <blimpID>"
fi

