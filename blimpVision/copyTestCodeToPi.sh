#!/bin/bash
if [ "$1" != "" ]; then
	hostname=$1
	echo ">>Copying workspace source code files to pi@$hostname"
	scp -r ./piTestVision pi@$hostname:/home/pi/
	ssh pi@$hostname 'make -C /home/pi/piTestVision'
	echo ">>Done."
else
	echo "Usage: copyCodeToPi.sh [hostname]"
fi

