#!/bin/bash
if [ "$1" != "" ]; then
	hostname=$1
	echo "Killing program."
	ssh pi@$hostname 'sudo killall -9 piTestCompile'
	echo "Waiting 3 seconds before restarting..."
	sleep 3
	echo "Starting program."
	ssh pi@$hostname "./piTestCompileSrc/build/piTestCompile $blimpID"
else
	echo "Usage: restartProgram.sh [hostname]"
fi

