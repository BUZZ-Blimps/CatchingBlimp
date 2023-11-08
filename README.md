# Catching Blimp Repository

## Links
- [Current Teensy Code](/Summer2023/BlimpV8/BlimpV8_Teensy/)
- [Vision Scripts](/scripts/Vision/)
- [Older Documentation](/Summer2023/README.md)

## Vision
- To start vision, go to [/scripts/Vision/](/scripts/Vision/).
- To edit which nodes are run for each blimp, edit script ```launchVisionGeneric.launch.xml```
    - This script uses many include statements, which each run a ros2 launch file
- To edit which blimps are started, edit script ```launchVision.launch.xml```
    - This script references a new instance of ```launchVisionGeneric``` for each blimp, passing in the relevant parameters per blimp (blimp name, calibration file name)
- To start the chosen blimps and nodes, run script ```./launchVision.sh```
    - This automatically sources any necessary directories and then starts ```launchVision.launch.xml```
