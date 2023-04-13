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

#define HEARTBEAT_PERIOD        0.2

#define MAXLINE                 1024


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

        // General
        ProgramData* programData;

        // Streaming
        pthread_t streaming_thread, BSFeedback_thread, MLFeedback_thread;

        int stream_socket_fd;
        struct sockaddr_in stream_server_addr;
            //const char* stream_server_ip = "192.168.0.202";
        const char* stream_server_ip = "192.168.0.200";
            //const char* stream_server_ip = "127.0.0.1";
        const int stream_server_port = 12345;

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

        const int MAX_DGRAM = pow(2, 16);
        const int MAX_IMAGE_DGRAM = MAX_DGRAM - 64;

        // Base station communication
        autoState mode;
        pthread_mutex_t mutex_mode;


        // UDP
        bool readUDP(string* retMessage = nullptr, string* target = nullptr, string* source = nullptr, string* flag = nullptr);
        void sendUDPRaw(string target, string source, string flag, string message);
        void sendUDP(string flag, string message);
        void sendUDP(string message);

    public:
        void init(ProgramData* programData);
        string getIPAddress();

        // ============================== SERIAL ==============================
        void initSerial();
        void sendSerial(std::string message);
        char readSerial();

        // ============================== UDP ==============================
        void initUDPReceiver();
        void initUDPSender();
        
        // Protocol
        bool validTarget(string targetID);
        void parseAutonomousMessage(string message, BSFeedbackData* BSFB);
        void parseManualMessage(string message, BSFeedbackData* BSFB);
        void parseBarometer(string message, BSFeedbackData* BSFB);
        //void establishBlimpID(); //EXTREMELY DEPRECATED

        // ============================== STREAMING ==============================
        void initStreamSocket();
        void setStreamFrame(Mat frame);
        BSFeedbackData getBSFeedback();
        MLFeedbackData getMLData();

        static void* staticStreamingThread_start(void* arg); 
        static void* staticBSFeedbackThread_start(void* arg);
        static void* staticMLFeedbackThread_start(void* arg);
        void streamingThread_loop();
        void BSFeedbackThread_loop();
        void MLFeedbackThread_loop();

        void setMode(autoState newMode);
};