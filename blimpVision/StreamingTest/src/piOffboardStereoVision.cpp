//C++ includes
#include <algorithm>
#include <cmath>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <iostream>
#include <iterator>
#include <list>
#include <unordered_map>
#include <string>
#include <vector>

//C includes
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <libv4l2.h>
#include <linux/videodev2.h>
#include <math.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

//OpenCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>

#include "piOffboardStereoVision.h"

using namespace std;
using namespace cv;

//==================== HELPER FUNCTIONS ====================//

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

void stop_program(int signal){
	programData.program_running = false;
	piComm.end();
	cameraHandler.end();
}

//==================== MAIN THREAD ====================//
int main(int argc, char** argv) {
	// Create struct to hold parameters set as command line arguments
	ProgramData programData;
	programData.program_running = true;

	// Assign interrupt signals
	signal(SIGINT, stop_program);
	signal(SIGABRT, stop_program);
	signal(SIGKILL, stop_program);
	signal(SIGTERM, stop_program);
	signal(SIGTSTP, stop_program);

	// START COMMUNICATION
	piComm.init(&programData);

	// INIT CAMERA
	cameraHandler.init(&piComm, &programData);

	// INIT COMPUTER VISION
	string srcDir = findSourceDir(argv); // Required for reading calibration files
	computerVision.init(&programData, srcDir, &piComm);

	clock_t last = 0;

	//for serial in case teensy restarts
	clock_t lastMsgTime = 0;
	clock_t lastSerial = 0;

    while (programData.program_running) {
		clock_t currentTime = clock();

		// Get recent frames
		Mat frame_L, frame_R;
		bool recentFrames = cameraHandler.getRecentFrames(&frame_L, &frame_R);
		if(!recentFrames) continue;

		// Do computer vision
		computerVision.update(frame_L, frame_R, piComm.getMode(), orange);
		int quad = computerVision.getQuad();

		// Get feedback from machine learning
		MLFeedbackData MLFeedback = piComm.getMLData();

		float time = (float)(currentTime-last)/(float)CLOCKS_PER_SEC;
		//cout << "Vision Compute Time: " << time << endl;
		//cout << "Vision Compute Rate: " << 1/time << endl;
		last = currentTime;

		computerVision.left_correct.copyTo(annotatedFrame);
		videoSaver.writeFrame(frame_L);

		//Select largest target for blimp depending on state:
		std::vector<std::vector<float> > target;

		autoState currMode = piComm.getMode();
		if (currMode == searching || currMode == approach || currMode == catching) {
			target = computerVision.getTargetBalloon();
		} else if (currMode == goalSearch || currMode == approachGoal || currMode == scoringStart) {
			target = computerVision.getTargetGoal();
		}

    } //end while (main program loop)

	stop_program(0);
    return 0;
}
