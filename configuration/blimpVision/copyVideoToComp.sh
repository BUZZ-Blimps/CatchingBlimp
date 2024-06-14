#!/bin/bash
if [ "$1" != "" ]; then
	hostname=$1
	echo ">>Copying video from pi@$hostname"
	scp pi@$hostname:/home/pi/outputVideo.avi ./Videos/
	echo ">>Done."
else
	echo "Usage: copyVideoToComp.sh [hostname]"
fi

