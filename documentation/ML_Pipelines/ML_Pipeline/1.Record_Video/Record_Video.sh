#!/bin/bash

cd ../6.Run_This_Shell_Script/Computation/piRecordVideo
rm -r build
mkdir build
make
cd build
./piRecordVideo
mv recording.avi ../../../../2.Divide_Video_To_Images
cd ../../../../2.Divide_Video_To_Images
echo ""
echo "Recording is located in directory $PWD"
echo "Recording Name: recording.avi"
echo ""
