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

if [ "$#" -lt 1 ]; then
    echo "Usage: $0 <OrangePi Number> [Path to firmware.hex]"
    exit 1
fi

orangePiNumber=$1
defaultHexPath=".pio/build/teensy40/firmware.hex"

# Check if a path to firmware.hex was provided, otherwise use default
if [ "$#" -eq 2 ]; then
    hexFilePath=$2
else
    if [ -f "$defaultHexPath" ]; then
        hexFilePath=$defaultHexPath
    else
        echo "No firmware.hex path provided and default file does not exist."
        exit 1
    fi
fi

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

echo "Stopping the microros service before flashing..."
sshpass -p buzzblimps ssh -o StrictHostKeyChecking=no opi@192.168.0.10$orangePiNumber "echo buzzblimps | sudo -S systemctl stop microros"

echo "Ensuring the teensyCode directory exists on the Orange Pi..."
sshpass -p buzzblimps ssh -o StrictHostKeyChecking=no opi@192.168.0.10$orangePiNumber "mkdir -p /home/opi/teensy_loader_cli/teensyCode/"

echo "Removing the existing firmware.hex file from the Orange Pi..."
sshpass -p buzzblimps ssh -o StrictHostKeyChecking=no opi@192.168.0.10$orangePiNumber "rm -f /home/opi/teensy_loader_cli/teensyCode/firmware.hex"

echo "Copying the new firmware.hex file to the appropriate Orange Pi..."
sshpass -p buzzblimps scp -o StrictHostKeyChecking=no "$hexFilePath" opi@192.168.0.10$orangePiNumber:/home/opi/teensy_loader_cli/teensyCode/

echo "Flashing the Teensy on the Orange Pi..."
sshpass -p buzzblimps ssh -o StrictHostKeyChecking=no opi@192.168.0.10$orangePiNumber "/home/opi/teensy_loader_cli/teensy_loader_cli -mmcu=TEENSY40 -s -w -v /home/opi/teensy_loader_cli/teensyCode/firmware.hex"

echo "Script completed successfully."
