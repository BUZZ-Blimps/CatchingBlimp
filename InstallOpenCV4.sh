#!/bin/bash

# Put entire installation inside folder
mkdir -p InstallOpenCV && cd InstallOpenCV

# Use instructions from https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html
# Install minimal prerequisites (Ubuntu 18.04 as reference)
sudo apt update && sudo apt install -y cmake g++ wget unzip
# Download and unpack sources
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.5.5.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.5.5.zip
unzip opencv.zip
unzip opencv_contrib.zip
# Create build directory and switch into it
mkdir -p build && cd build
# Configure
cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.5.5/modules ../opencv-4.5.5
# Build
cmake --build .

# Follow up steps, executed from build directory
sudo make install

# Create .gitignore to ignore everything. 
cd ..
echo "*" > .gitignore
# There's a good chance the .gitignore will be hidden, check in command line with: ls -al

## LINKER STUFF FOUND AT /usr/local/include/opencv4/**
