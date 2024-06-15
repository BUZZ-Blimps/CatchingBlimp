#!/bin/bash

cd ../3.Label_Images/Images
rm *.png
cd ../../2.Divide_Video_To_Images
ffmpeg -i *.avi im%05d.png
mv *.png ../3.Label_Images/Images
cd ../3.Label_Images
echo ""
echo "Images Created"
echo "Image Folder is located in directory $PWD"
echo ""
