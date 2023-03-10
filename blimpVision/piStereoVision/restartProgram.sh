#!/bin/bash
if [ "$1" != "" ] && [ "$2" != "" ]; then
	hostname=$1
	blimpID=$2
	echo "Killing program."
	ssh pi@$hostname 'sudo killall -9 piStereoVision'
	echo "Waiting 6 seconds before restarting..."
	sleep 6
	echo "Starting program."
	ssh pi@$hostname "./piStereoVisionSrc/build/piStereoVision $blimpID"
else
	echo "Usage: restartProgram.sh [hostname] [blimpID]"
fi

