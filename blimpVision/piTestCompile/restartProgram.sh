#!/bin/bash
if [ "$1" != "" ]; then
	hostname=$1
	echo "Starting program."
	ssh pi@$hostname "./piTestCompileSrc/build/piTestCompile $blimpID"
else
	echo "Usage: restartProgram.sh [hostname]"
fi

