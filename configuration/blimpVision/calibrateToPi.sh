#!/bin/bash
if [ "$1" != "" ]; then
	hostname=$1
	echo ">>Deleting previous folder."
	rm -r takeImagesCalibrate/tempImages
	
	echo ">>Creating tempImages folder."
	mkdir takeImagesCalibrate/tempImages
	
	echo ">>Starting calibration process: Beginning to take images."
	./takeImagesCalibrate/build/takeImagesCalibrate c
	echo ">>Finished calibration process."
	
	echo ">>Killing piStereoVision on pi."
	ssh pi@$hostname 'sudo killall -9 piStereoVision'
	
	echo ">>Copying resultant calibration data to pi."
	scp ./takeImagesCalibrate/build/stereo_rectify_maps.xml pi@$hostname:/home/pi/piStereoVision
	
	echo ">>Done."
else
	echo "Usage: calibrateToPi.sh [hostname]"
fi

