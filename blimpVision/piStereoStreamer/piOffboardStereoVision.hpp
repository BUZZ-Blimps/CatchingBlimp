/*
 * piOffboardStereoVision.hpp
 *
 *  Created on: Nov 10, 2022
 *      Author: willie
 */
#ifndef PIOFFBOARDSTEREOVISION_HPP_
#define PIOFFBOARDSTEREOVISION_HPP_

//==================== CONSTANTS ====================
#define HEARTBEAT_PERIOD    0.1

//Video streaming
const char* stream_server_ip = "192.168.0.202";
const int   stream_server_port = 12345;

//Communication
#define UDP_IP				"239.255.255.250"
#define UDP_PORT			1900
#define UDPTimeout			5

const int MAX_DGRAM = pow(2, 16);
const int MAX_IMAGE_DGRAM = MAX_DGRAM - 64;

const char *stereo_cal_path = "/home/pi/piOffboardStereoVision/stereo_rectify_maps240p.xml";
//const char *stereo_cal_path = "/home/willie/CatchingBlimp/blimp-workspace/piOffboardStereoVision/src/stereo_rectify_maps240p.xml";

#define MAXLINE 1024

//Camera
#define CAMERA_WIDTH	320
#define CAMERA_HEIGHT	240

#define CV_WIDTH		320
#define CV_HEIGHT		240

#define RECT_WIDTH		320
#define RECT_HEIGHT		240

//Stereo
#define DISP_WIDTH		160
#define DISP_HEIGHT		120

#define CONVERSION		0.15

#define PRE_FILTER_SIZE	7
#define PRE_FILTER_CAP	2
#define UNIQUENESS_RATIO	5


#define LAMBDA			17.8
#define SIGMA			5.0

//Colors
#define B_CORRECTION	Scalar(29,7,15)
#define B_MIN			Scalar(46,0,0)
#define B_MAX			Scalar(96,74,213)

#define ORANGE_G_CORRECTION    Scalar(0,47,0)
#define ORANGE_G_MIN           Scalar(13,0,0)
#define ORANGE_G_MAX           Scalar(24,255,255)

#define YELLOW_G_CORRECTION	    Scalar(48,0,0)
#define YELLOW_G_MIN			Scalar(25,0,0)
#define YELLOW_G_MAX			Scalar(63, 255, 255)

#define G_POLY_APPROX_E          0.01
#define GOAL_INNER_CONTOUR_SCALE 0.7
#define GOAL_CONFIRM			 6000

#define E_ITER 1
#define D_ITER 1
#define E_SIZE 1
#define D_SIZE 1

#define GOAL_DILATION_ADJUST 4

#define MIN_AREA		50
#define SIZE_RATIO		3

#define AVOID_DIST		70
#define AVOID_AREA		6000

#define ML_CLASS_BALLOON		0
#define ML_CLASS_ORANGEGOAL		1
#define ML_CLASS_YELLOWGOAL		2

//==================== GLOBAL VARIABLES (GOD FORBID) ====================//
int blimpID = -1;
int cap_device_id = 0;
int teensyState;
bool program_running = true;
bool autonomous = false;

clock_t lastReceivedTargetsTime;
clock_t lastReceivedTargetsTimeCopy;
bool lastReceivedIsNew = false;
double receivedTargetTimeout = 1; //seconds

bool scoreInOrange = false;
bool selfIsBlue = false;

//Running modes
bool debugMode = false;
bool verboseMode = false;
bool streamOnlyMode = false;
bool annotatedMode = false;
bool disableSerialMode = false;
bool printJSONMode = false;

cv::Mat annotatedFrame;
bool annotatedFrameReady = false;

std::string msgTemp = "";

float barometerData = 0;

std::vector<float> recentMotorCommands;

//Low res frames for stereo processing
cv::Mat lt_frame_lowres, rt_frame_lowres;

int stream_socket_fd;
struct sockaddr_in stream_server_addr;

pthread_t bs_udp_thread, stream_thread, stream_fb_thread;
long bs_udp_thread_id, stream_thread_id, stream_fb_thread_id;

//Timing
clock_t lastTP = 0;
std::vector<clock_t> times;
std::vector<std::string> flags;

clock_t lastUDPReceived = 0;

//Video recording
int framesLeftToRecord;
std::string outputVideo_fileName = "outputVideo.avi";
double outputVideo_fps = 30;

float lastBaroMessageTime = 0.0;

//Teleplot teleplot("127.0.0.1");

std::string groupAddress = UDP_IP;
char* group = &groupAddress[0];
int port = 	UDP_PORT;

int recSocketFD, sendSocketFD;
struct sockaddr_in addrRec, addrSend;

//State machine enums
enum object {
    balloon,
    blimpB,
    blimpR,
    goalO,
    goalY,
    first = balloon,
    last = goalY
};

enum autoState {
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

enum blimpType {
	blue,
	red
};

enum goalType {
	orange,
	yellow
};

int blimpColor = blue;
int goalColor = orange;
int mode = searching;

//==================== FUNCTION HEADERS ====================
//cv::VideoCapture openCamera(int camIndex, int camWidth, int camHeight);
//bool parseArguments(int argc, char** argv);
//std::vector<cv::Point> scaleContour(std::vector<cv::Point> contour, float scale);
//std::vector<std::vector<float> > getObjects(int type, cv::Mat mask);

//bool initPython();
//string runYolo(Mat& frame);

//==================== COMMUNICATION FUNCTION HEADERS ====================
//void initSerial();
//void sendSerial(std::string message);
//char readSerial();
//void initUDPReceiver();
//void initUDPSender();
//bool readUDP(std::string* retMessage = nullptr, std::string* target = nullptr, std::string* source = nullptr, std::string* flag = nullptr);
//void sendUDPRaw(std::string target, std::string source, std::string flag, std::string message);
//void sendUDP(std::string flag, std::string message);
//void sendUDP(std::string message);
//void establishBlimpID();
//void plotUDP(std::string varName, float varValue);

//==================== HELPER FUNCTION HEADERS ====================
//float getFPS();
//void benchmarkFirst(std::string flag);
//void benchmark(std::string flag);
//void benchmarkPrint();
//void delay(double delaySeconds);
//void saveToVideo(cv::Mat frame);

//==================== GLOBAL VARIABLES (GOD FORBID) ====================

#endif /* PIOFFBOARDSTEREOVISION_HPP_ */
