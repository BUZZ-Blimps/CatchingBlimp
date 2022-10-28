#!/bin/bash
if [ "$1" != "" ]; then
	hostname=$1
	echo "Killing program."
	ssh pi@$hostname 'sudo killall -9 piStereoVision'
	echo "Waiting 6 seconds before restarting..."
	sleep 6
	echo "Starting program."
	ssh pi@$hostname './piStereoVision/build/piStereoVision'
else
	echo "Usage: restartProgram.sh [hostname]"
fi

