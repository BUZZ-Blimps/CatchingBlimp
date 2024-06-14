#!/bin/bash
if [ "$1" != "" ]; then
	hostname=$1
	echo ">>Copying workspace source code files to pi@$hostname"
	scp -r ./piRecordVideo pi@$hostname:/home/pi/
	ssh pi@$hostname 'make -C /home/pi/piRecordVideo'
	echo ">>Done."
else
	echo "Usage: copyRecordToPi.sh [hostname]"
fi

