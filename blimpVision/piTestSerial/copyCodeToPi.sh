#!/bin/bash
if [ "$1" != "" ]; then
	hostname=$1
	
	echo ">>Removing old workspace from pi@$hostname"
	ssh pi@$hostname 'rm -rf /home/pi/piTestSerialSrc'
	
	echo ">>Copying workspace source code files to pi@$hostname"
	scp -r ./piTestSerialSrc pi@$hostname:/home/pi/
	
	echo "Compiling code."
	ssh pi@$hostname 'make -C /home/pi/piTestSerialSrc'
	
	echo ">>Done."
else
	echo "Usage: copyCodeToPi.sh [hostname]"
fi

