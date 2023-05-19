#pragma once
#include <Arduino.h>
#include "Quack.h"
#include <vector>


class UDPComm {
private:
	String identifier = ":)";
	String blimpID;

public:
	Quack<String> packets[4];

	void init();
	//String getBlimpID(); // DEPRECATED - use getIPAddress() instead

	void send(String message);
	void send(String targetID, String flag, String message);
	bool readPackets();
	void packetClear();
	
  	String packetMove;
  	unsigned long last_message_recieved = 0;

	String packetGetTargetID(String packet);
	String packetGetSourceID(String packet);
	String packetGetFlag(String packet);
	String packetGetMessage(String packet);
  	std::vector<String> packetMoveGetInput();
};
