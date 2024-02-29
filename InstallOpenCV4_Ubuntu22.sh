### References
# https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html
# https://stackoverflow.com/questions/66133421/unable-to-display-window-via-cvimshow-what-am-i-missing

# Install prerequisites
sudo apt update && sudo apt install -y cmake g++ wget unzip

sudo apt install libgtk2.0-dev

sudo apt install build-essential cmake git pkg-config libgtk-3-dev \ 
libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \ 
libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev \ 
gfortran openexr libatlas-base-dev python3-dev python3-numpy \ 
libtbb2 libtbb-dev

sudo apt install v4l-utils

sudo apt install libcanberra-gtk-module

# Download and unpack sources
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.x.zip
unzip opencv.zip
unzip opencv_contrib.zip

# Create build directory and switch into it
mkdir -p build && cd build

# Configure
cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.x/modules ../opencv-4.x

# Build
cmake --build .

# Install
sudo make install

# Create .gitignore to ignore everything. 
cd ..
echo "*" > .gitignore
# There's a good chance the .gitignore will be hidden, check in command line with: ls -al
