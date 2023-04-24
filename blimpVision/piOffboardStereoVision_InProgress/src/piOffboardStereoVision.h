#pragma once

#include <string>

using namespace std;

// Libraries
#include "PiComm.h"
#include "ComputerVision.h"
#include "CameraHandler.h"
#include "VideoSaver.h"
#include "Util.h"
#include "Teleplot.h"

// Defines
#define ML_CLASS_BALLOON		0
#define ML_CLASS_ORANGEGOAL		1
#define ML_CLASS_YELLOWGOAL		2

#define UDPTimeout			    5

// Objects
PiComm piComm;
ComputerVision computerVision;
CameraHandler cameraHandler;
ProgramData programData;
VideoSaver videoSaver("outputVideo.avi", 30, false);

// Constants
double receivedTargetTimeout = 1; //seconds

// Variables
Mat annotatedFrame;
string msgTemp;
float baroData;
