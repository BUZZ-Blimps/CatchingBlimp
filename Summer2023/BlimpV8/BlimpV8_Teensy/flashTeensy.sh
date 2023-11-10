#/bin/bash

if ( [ "$#" -gt 1 ] || [ "$#" -lt 1 ] );
then
    echo "Invalid number of arguments"
elif ( [ "$1" -lt 1 ] || [ "$1" -gt 6 ] );
then
    echo "Invalid input value"
elif ( [ "$#" -eq 1 ] );
then

    # Very Slow
    #sshpass -p 1234 ssh orangepi$1@orangepi$1 ./stopMicroros.sh

    sed -i '/std::string blimpNameSpace = /s/^\/\?\/\?/\/\//' "src/catching_blimp.h"
    if [ "$1" = "1" ]; then
    	sed -i "/std::string blimpNameSpace = \"BurnCreamBlimp\"/s/^\/\///" "src/catching_blimp.h"
    elif [ "$1" = "2" ]; then
    	sed -i "/std::string blimpNameSpace = \"SillyAhBlimp\"/s/^\/\///" "src/catching_blimp.h"
    elif [ "$1" = "3" ]; then
    	sed -i "/std::string blimpNameSpace = \"TurboBlimp\"/s/^\/\///" "src/catching_blimp.h"
    elif [ "$1" = "4" ]; then
    	sed -i "/std::string blimpNameSpace = \"GameChamberBlimp\"/s/^\/\///" "src/catching_blimp.h"
    elif [ "$1" = "5" ]; then
    	sed -i "/std::string blimpNameSpace = \"FiveGuysBlimp\"/s/^\/\///" "src/catching_blimp.h"
    elif [ "$1" = "6" ]; then
    	sed -i "/std::string blimpNameSpace = \"SuperBeefBlimp\"/s/^\/\///" "src/catching_blimp.h"
    fi

    #Build firmware.hex file
    platformio run --environment teensy40

    #Copy firmware.hex file to Orange Pi
    scp /home/corelab/GitHub/CatchingBlimp/Summer2023/BlimpV8/BlimpV8_Teensy/.pio/build/teensy40/firmware.hex orangepi$1@orangepi$1:/home/orangepi$1/teensyCode/

    #Enter into the Orange Pi
    #Flash the Teensy (For some reason we have to do this twice)
    #Must press the button if the Teensy has never been flashed before
    ssh orangepi$1@orangepi$1 "/home/orangepi$1/teensy_loader_cli/teensy_loader_cli -mmcu=TEENSY40 -s -w -v /home/orangepi$1/teensyCode/firmware.hex; /home/orangepi$1/teensy_loader_cli/teensy_loader_cli -mmcu=TEENSY40 -s -w -v /home/orangepi$1/teensyCode/firmware.hex"

    # Doesn't work without password
    #sshpass -p 1234 ssh orangepi$1@orangepi$1 ./restartMicroros.sh
fi

#WARNING: IF THE BLIMP DOES NOT APPEAR ON THE BASE STATION, THEN SSH INTO THE PI AND CHECK THAT
#	  MICROROS IS RUNNING
