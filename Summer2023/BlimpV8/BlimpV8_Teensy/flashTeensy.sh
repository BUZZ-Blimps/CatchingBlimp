#!/bin/bash

echo "Starting the script..."

# Check if sshpass is installed, if not, install it
if ! command -v sshpass &> /dev/null; then
    echo "sshpass could not be found, attempting to install it..."
    sudo apt-get update && sudo apt-get install -y sshpass
    if [ $? -ne 0 ]; then
        echo "Failed to install sshpass, exiting."
        exit 1
    fi
fi

if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <OrangePi Number> <Path to firmware.hex>"
    exit 1
fi

orangePiNumber=$1
hexFilePath=$2

echo "Checking Orange Pi number validity..."
if [ "$orangePiNumber" -lt 1 ] || [ "$orangePiNumber" -gt 6 ]; then
    echo "Invalid Orange Pi number: $orangePiNumber"
    exit 1
fi

echo "Updating the namespace in the source file..."
sed -i '/std::string blimpNameSpace = /s/^\/\?\/\?/\/\//' "src/catching_blimp.h"
case "$orangePiNumber" in
    1) sed -i "/std::string blimpNameSpace = \"BurnCreamBlimp\"/s/^\/\///" "src/catching_blimp.h" ;;
    2) sed -i "/std::string blimpNameSpace = \"SillyAhBlimp\"/s/^\/\///" "src/catching_blimp.h" ;;
    3) sed -i "/std::string blimpNameSpace = \"TurboBlimp\"/s/^\/\///" "src/catching_blimp.h" ;;
    4) sed -i "/std::string blimpNameSpace = \"GameChamberBlimp\"/s/^\/\///" "src/catching_blimp.h" ;;
    5) sed -i "/std::string blimpNameSpace = \"FiveGuysBlimp\"/s/^\/\///" "src/catching_blimp.h" ;;
    6) sed -i "/std::string blimpNameSpace = \"SuperBeefBlimp\"/s/^\/\///" "src/catching_blimp.h" ;;
esac

platformio run --environment teensy40

echo "Ensuring the teensyCode directory exists on the Orange Pi..."
sshpass -p buzzblimps ssh -o StrictHostKeyChecking=no opi@192.168.0.10$orangePiNumber "mkdir -p /home/opi/teensy_loader_cli/teensyCode/"

echo "Copying the firmware.hex file to the appropriate Orange Pi..."
sshpass -p buzzblimps scp -o StrictHostKeyChecking=no "$hexFilePath" opi@192.168.0.10$orangePiNumber:/home/opi/teensy_loader_cli/teensyCode/

echo "Flashing the Teensy on the Orange Pi..."
sshpass -p buzzblimps ssh -o StrictHostKeyChecking=no opi@192.168.0.10$orangePiNumber "/home/opi/teensy_loader_cli/teensy_loader_cli -mmcu=TEENSY40 -s -w -v /home/opi/teensy_loader_cli/teensyCode/firmware.hex"

echo "Script completed successfully."
