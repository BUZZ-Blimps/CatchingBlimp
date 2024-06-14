#include <Arduino.h>
#include "BlimpClock.h"


void BlimpClock::setFrequency(double frequency) {
	lastTime = 0;
	milliDelay = round(1000 / frequency);
}

bool BlimpClock::isReady() {
	long currentTime = millis();
	if (currentTime - lastTime >= milliDelay) {
		lastTime = currentTime;
		return true;
	}else {
		return false;
	}
}
