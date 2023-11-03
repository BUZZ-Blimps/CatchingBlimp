#/bin/bash

if ( [ "$#" -gt 1 ] || [ "$#" -lt 1 ] );
then
    echo "Invalid number of arguments"
elif ( [ "$1" -lt 1 ] || [ "$1" -gt 6 ] );
then
    echo "Invalid input value"
elif ( [ "$#" -eq 1 ] );
then

    #Build firmware.hex file
    platformio run --environment teensy40

    #Copy firmware.hex file to Orange Pi
    scp /home/corelab/GitHub/CatchingBlimp/Summer2023/BlimpV8/BlimpV8_Teensy/.pio/build/teensy40/firmware.hex orangepi$1@orangepi$1:/home/orangepi$1/teensyCode/

    #Enter into the Orange Pi
    #Flash the Teensy (For some reason we have to do this twice)
    sshpass -p 1234 ssh orangepi$1@orangepi$1 "/home/orangepi$1/teensy_loader_cli/teensy_loader_cli -mmcu=TEENSY40 -s -w -v /home/orangepi$1/teensyCode/firmware.hex; /home/orangepi$1/teensy_loader_cli/teensy_loader_cli -mmcu=TEENSY40 -s -w -v /home/orangepi$1/teensyCode/firmware.hex"
fi