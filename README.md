# Catching Blimp Repository

## Links
- [Current Teensy Code](/Summer2023/BlimpV8/BlimpV8_Teensy/)
- [Vision Scripts](/scripts/Vision/)
- [Older Documentation]()

## Vision
- To start vision, go to [/scripts/Vision/](/scripts/Vision/).
- To edit which nodes are run for each blimp, edit script ```launchVisionGeneric.launch.xml```
    - This script uses ```include``` statements
- Run script ```./launchVision.sh```
    - This automatically sources any necessary directories
- To edit which blimps are run