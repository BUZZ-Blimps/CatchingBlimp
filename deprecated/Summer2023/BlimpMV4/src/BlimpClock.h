#pragma once

class BlimpClock {
private:
	long lastTime;
	int milliDelay;

public:
	void setFrequency(double frequency);
	bool isReady();
};
