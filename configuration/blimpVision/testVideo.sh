#!/bin/bash
if [ "$1" != "" ]; then
	hostname=$1
	echo ">>Copying workspace source code files to pi@$hostname"
	scp -r ./testVideo pi@$hostname:/home/pi/
	
	echo ">>Killing program."
	ssh pi@$hostname 'sudo killall -9 testVideo'
	
	echo ">>Compiling code."
	ssh pi@$hostname 'make -C /home/pi/testVideo'
	
	echo ">>Starting program."
	ssh pi@$hostname './testVideo/build/testVideo'
	
	echo ">>Copying video."
	scp pi@$hostname:/home/pi/outputVideo.avi ./Videos/
else
	echo "Usage: testVideo.sh [hostname]"
fi

