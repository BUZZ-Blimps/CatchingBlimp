#pragma once

// ============================== INCLUDES ==============================
#include <string>

// ============================== DEFINES ==============================
#define SERIAL_ATTEMPT_DELAY    1 //second

#define UDP_IP				    "239.255.255.250"
#define UDP_PORT			    1900
#define UDPTimeout			    5

// ============================== CLASS ==============================

class PiComm{
    private:
        int serial;
        int blimpID;

    public:

    void setBlimpID(int newBlimpID);

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

};