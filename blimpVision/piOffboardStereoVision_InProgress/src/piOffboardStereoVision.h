#pragma once

#include <string>

// Libraries
#include "PiComm.h"
#include "ComputerVision.h"
#include "CameraHandler.h"
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

// Constants
double receivedTargetTimeout = 1; //seconds

// Variables
autoState mode;
Mat annotatedFrame;
std::string msgTemp;

