#!/bin/bash
if [ "$1" != "" ]; then
	hostname=$1
	echo ">>Copying workspace source code files to pi@$hostname"
	scp -r ./testCompilePi pi@$hostname:/home/pi/
	
	ssh pi@$hostname 'mkdir -p /home/pi/testCompilePi/build && make -C /home/pi/testCompilePi'
	echo ">>Done."
else
	echo "Usage: copyTestCodeToPi.sh [hostname]"
fi

