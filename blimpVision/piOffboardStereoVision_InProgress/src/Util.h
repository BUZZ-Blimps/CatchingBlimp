#pragma once

#include <vector>
#include "json.hpp"
using json = nlohmann::json;

enum object {
    balloon,
    blimpB,
    blimpR,
    goalO,
    goalY,
    first = balloon,
    last = goalY
};

enum autoState{
	searching,
	approach,
	catching,
	caught,
	goalSearch,
	approachGoal,
	scoringStart,
	shooting,
	scored
};

enum blimpType{
	blue,
	red
};

enum goalType{
	orange,
	yellow
};

struct BSFeedbackData{
	clock_t lastUDPReceived = 0;
	float lastBaroMessageTime = 0;

	bool autonomous;
	float barometerData;
	std::vector<float> recentMotorCommands;
	goalType goalColor;

	// Creates new BSFeedbackData but carries over all old times
	BSFeedbackData cloneTimes();
};

BSFeedbackData BSFeedbackData::cloneTimes(){
	BSFeedbackData newBSFB;

	newBSFB.lastUDPReceived = this->lastUDPReceived;
	newBSFB.lastBaroMessageTime = this->lastBaroMessageTime;

	return newBSFB;
}

struct MLFeedbackData{
	json lastReceivedTargets;
	clock_t lastReceivedTargetsTime;
};

struct ProgramData{
	bool verboseMode = false;
	bool streamOnlyMode = false;
	bool annotatedMode = false;
	bool disableSerialMode = false;
	bool printJSONMode = false;

	bool setBlimpID = false;
	string customBlimpID;

	bool setCaptureID = false;
	int customCaptureID;

	bool program_running = true;
};