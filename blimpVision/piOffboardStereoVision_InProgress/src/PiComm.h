#pragma once

// ============================== INCLUDES ==============================
#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>
#include <stdlib.h>
#include <vector>

#include "serialib.h"
#include "Util.h"

using namespace std;

// ============================== DEFINES ==============================
#define SERIAL_ATTEMPT_DELAY    1 //second

#define UDP_IP				    "239.255.255.250"
#define UDP_PORT			    1900
#define UDPTimeout			    5

#define FLAG_AUTONOMOUS         "A"
#define FLAG_MANUAL             "M"
#define FLAG_BAROMETER          "B"
#define FLAG_PARAMETER          "P"
#define FLAG_KILL               "K"
#define FLAG_TARGETGOAL         "TG"

// ============================== CLASS ==============================

class PiComm{
    private:
        // Blimp ID
        string blimpID;

        // Serial communication
        serialib serial;

        // UDP communication
        string groupAddress = UDP_IP;
        char* group = &groupAddress[0];
        int port = 	UDP_PORT;

        int sockRec;
        struct sockaddr_in addrRec;
        int sockSend;
        struct sockaddr_in addrSend;

        // Streaming
        pthread_t streaming_thread, BSFeedback_thread, MLFeedback_thread;

        int stream_socket_fd;
        struct sockaddr_in stream_server_addr;
            //const char* stream_server_ip = "192.168.0.202";
        const char* stream_server_ip = "192.168.0.200";
            //const char* stream_server_ip = "127.0.0.1";
        const int   stream_server_port = 12345;

        Mat frameToStream;
        pthread_mutex_t mutex_frameToStream;
        unsigned int newFrameNum;
        pthread_mutex_t mutex_newFrameNum;
        unsigned int prevFrameNum;
        void send_frame(Mat image);

        BSFeedbackData BSFeedback;
        pthread_mutex_t mutex_BSFeedback;
        
        MLFeedbackData MLFeedback;
        pthread_mutex_t mutex_MLFeedback; 


    public:
        void setBlimpID(string newBlimpID);
        string getIPAddress();

        // ============================== SERIAL ==============================
        void initSerial();
        void sendSerial(std::string message);
        char readSerial();

        // ============================== UDP ==============================
        void initUDPReceiver();
        void initUDPSender();
        bool readUDP(string* retMessage = nullptr, string* target = nullptr, string* source = nullptr, string* flag = nullptr);
        void sendUDPRaw(string target, string source, string flag, string message);
        void sendUDP(string flag, string message);
        void sendUDP(string message);
        
        // Protocol
        bool validTarget(string targetID);
        bool parseAutonomousMessage(string message, bool* newBaroDataValid, float* newBaroData, goalType* newGoalColor);
        bool parseManualMessage(string message, vector<float>* motorCommands, bool* newBaroDataValid, float* newBaroData, goalType* newGoalColor);
        bool parseBarometer(string message, float* newBaroData);
        //void establishBlimpID(); //EXTREMELY DEPRECATED

        // ============================== STREAMING ==============================
        void initStreamSocket();
        void setStreamFrame(Mat frame);
        BSFeedbackData getBSFeedback();
        MLFeedbackData getMLData();

        void* staticStreamingThread_start(void* arg); 
        void* staticBSFeedbackThread_start(void* arg);
        void* staticMLFeedbackThread_loop(void* arg);
        void streamingThread_loop();
        void BSFeedbackThread_loop();
        void MLFeedbackThread_loop();
};