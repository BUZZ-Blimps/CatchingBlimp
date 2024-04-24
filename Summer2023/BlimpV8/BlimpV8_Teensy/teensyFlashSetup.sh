#!/bin/bash

# Copy teensy_loader_cli directory
git clone https://github.com/PaulStoffregen/teensy_loader_cli.git

# Enter teensy_loader_cli directory
cd teensy_loader_cli.git

# Install dependencies
sudo apt install gcc
sudo apt install libusb-dev

# Create teensy_loader_cli bash script
# WARNING: POTENTIAL ERROR MAY OCCUR IF NOT USING LINUX
make

# Apply libusb-dev rules
# WARNING: SHOULD OUTPUT "SUCCESS" IF COMMAND WORKED
( sudo rm -f /tmp/00-teensy.rules /etc/udev/rules.d/00-teensy.rules /lib/udev/rules.d/00-teensy.rules && wget -O /tmp/00-teensy.rules https://www.pjrc.com/teensy/00-teensy.rules &&  sudo install -o root -g root -m 0664 /tmp/00-teensy.rules /lib/udev/rules.d/00-teensy.rules && sudo udevadm control --reload-rules && sudo udevadm trigger && echo "Success" )

# Create teensyCode directory to place firmware.hex file in
mkdir teensyCode

# Restart Pi
sudo reboot
