#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>
#include <ifaddrs.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/if_link.h>
#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/ximgproc.hpp>

//Lawson Code:
#include "EnumUtil.h"
//#include "ComputerVision.h"
#include "ComputerVision.h"

using namespace std;

string getIPAddress(){
	string IPAddress = "";
	string targetIfaName = "wlan0";
	int targetIfaFamily = AF_INET;
	string failureReturn = "";

	// Code from: https://man7.org/linux/man-pages/man3/getifaddrs.3.html
	struct ifaddrs *ifaddr;
	int s;
	char host[NI_MAXHOST];

	if (getifaddrs(&ifaddr) == -1) {
		cout << "Unable to get local IP addresses." << endl;
		return failureReturn;
	}

	/* Walk through linked list, maintaining head pointer so we
		can free list later. */

	for (struct ifaddrs *ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
		if (ifa->ifa_addr == NULL)
			continue;

		string ifaName = ifa->ifa_name;
		int ifaFamily = ifa->ifa_addr->sa_family;

		if(ifaName == targetIfaName && ifaFamily == targetIfaFamily){
			// getnameinfo ONLY IF: ifaFamily == AF_INET || ifaFamily == AF_INET6
			s = getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in), host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);
			if (s != 0) {
				cout << "getnameinfo() failed." << endl;
			}else{
				IPAddress = string(host);
				//cout << "Identified IP Address: " << IPAddress << endl;
			}
		}
	}

	freeifaddrs(ifaddr);
	return IPAddress;
}

string findSourceDir(char** argv){
	// Find current working directory
	int cwdBufSize = 200;
    char cwdBuf[cwdBufSize];
	getcwd(cwdBuf, cwdBufSize);
	string cwd(cwdBuf);

	// Find executable path
	string exePath(argv[0]);
	int lastSlash = exePath.find_last_of('/');
	exePath = exePath.substr(1, lastSlash-1); // Only want everything between the leading period and the final slash ("/")

	// Find total path
	string srcDirPath = cwd + exePath + "/..";
	return srcDirPath;
}

// Lawson Code:
//ComputerVision computerVision;
ComputerVision compVis;

autoState mode = searching;

//Variables
goalType goalColor = orange;

int main(int argc, char** argv) {
	cout << "Hello World!" << endl;

	string IPAddress = getIPAddress();
	cout << "IP Address: " << IPAddress << endl;

	// Lawson Code:

	// Old initialization
	//computerVision.init();
	//computerVision.readCalibrationFiles();

	// New Init
	compVis.init();
	string srcDir = findSourceDir(argv);
	compVis.readCalibrationFiles(srcDir);
	
	// Loop
	//namedWindow("imgL");
	cout << "PRESS SPACE TO PAUSE/RESUME VIDEO." << endl;
	while(true){
		
		/*
		//==================== OLD ====================
		//Update
		computerVision.update(mode, goalColor);

		//Select target for blimp depending on state:
		std::vector<std::vector<float> > target;

		//get largest balloon or goal depending on state
		if (mode == searching || mode == approach || mode == catching) {
			target = computerVision.getTargetBalloon();
		} else if (mode == goalSearch || mode == approachGoal || mode == scoringStart) {
			target = computerVision.getTargetGoal();
		}

		//Debugging
		//print target
		cout << mode << endl;
		

		for (int i = 0; i < target.size(); i++) {
			cout << "X: " << target[i][0] << endl;
			cout << "Y: " << target[i][1] << endl;
			cout << "Z: " << target[i][2] << endl;
			cout << "Area: " << target[i][3] << endl;
		}
		*/
		

		//==================== NEW ====================
		// compVis.update(searching, goalColor);
		// compVis.update_PI(mode, goalColor);
		float X, Y, Z, area, angle = 0;
		Mat left, right;
		compVis.getFrames(left, right);
		namedWindow("Left");
		imshow("Left", left);
		//compVis.tuneGoal(X, Y, Z, area, angle, left, right);
		// compVis.tuneBall(X, Y, Z, area, left, right);
		//compVis.tuneBall_Lawson(X, Y, Z, area, left, right);
		//compVis.getAvoidance(left, right);
	}

	
	return 0;
}
