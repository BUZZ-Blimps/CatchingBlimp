#pragma once

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <chrono>

#include "json.hpp"
using json = nlohmann::json;
using namespace std;

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
	BSFeedbackData cloneTimes(){
		BSFeedbackData newBSFB;

		newBSFB.lastUDPReceived = this->lastUDPReceived;
		newBSFB.lastBaroMessageTime = this->lastBaroMessageTime;

		return newBSFB;
	}

	std::string print(){
		std::string printString = "";
		
		printString += "LastUDPReceived: " + to_string(lastUDPReceived) + "\n";
		printString += "lastBaroMsgTime: " + to_string(lastBaroMessageTime) + "\n";
		printString += "Autonomous: " + to_string(autonomous) + "\n";
		printString += "BaroData: " + to_string(barometerData) + "\n";
		printString += "recentMotorCommands: ";
		for(int i=0; i<recentMotorCommands.size(); i++) printString += to_string(recentMotorCommands[i]) + ", ";
		printString += "\n";
		printString += "GoalColor: " + to_string(goalColor) + "\n";

		return printString;
	}
};

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
	bool printBaroMode = false;
	bool disableStreamMode = false;
	bool forceStreamMode = false;

	bool setBlimpID = false;
	std::string customBlimpID;

	bool setCaptureID = false;
	int customCaptureID;

	bool program_running = true;
	bool autonomous = false;
};

struct NamedMatPtr{
	cv::Mat* mat;
	std::string name;

	NamedMatPtr(cv::Mat* mat, std::string name){
		this->mat = mat;
		this->name = name;
	}
};