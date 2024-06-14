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

//Lawson Code:
#include "EnumUtil.h"
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

// Lawson Code:
ComputerVision computerVision;

autoState mode = searching;

//Variables
goalType goalColor = orange;

int main() {
	cout << "Hello World!" << endl;

	string IPAddress = getIPAddress();
	cout << "IP Address: " << IPAddress << endl;

	// Lawson Code:

	//initialization
	computerVision.init();
	computerVision.readCalibrationFiles();

	// Loop
	while(true){
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
		

		//==================== NEW ====================
	}

	
	return 0;
}
