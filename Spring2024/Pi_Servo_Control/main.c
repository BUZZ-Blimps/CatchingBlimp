#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <stdbool.h>

#include <wiringPi.h>

typedef struct {
	unsigned int ccr;		// Capture/Compare Register (Duty Cycle)
	unsigned int arr;		// Auto-Reload Register (Period)
	unsigned int div;
	unsigned int div_stepping;
} pwm_info;

#define PIN	2

// Far ranges = [70,528]
const int pwm_lower = 80;
const int pwm_upper = 510;

static pwm_info pwm_info_t;

int main (int argc, char *argv[]){
	pwm_info_t.ccr = 500;
	pwm_info_t.arr = 1000;
	pwm_info_t.div = 120;
	pwm_info_t.div_stepping = 2;

	wiringPiSetup();
	pinMode(PIN, PWM_OUTPUT);

	pwmSetRange(PIN, pwm_info_t.arr);
	pwmSetClock(PIN, pwm_info_t.div);
	pwmWrite(PIN, pwm_info_t.ccr);

	printf("Zeroing in 1 second...\n");
	delay(1000);
	pwmWrite(PIN, pwm_lower);
	delay(1000);

	while(true){
		printf("Sweeping up in 1 second...\n");
		delay(1000);
		for(int i=pwm_lower; i<=pwm_upper; i++){
			int val = i;
			printf("PWM Val: %d\n", val);
			pwmWrite(PIN, val);
			delay(5);
		}
		delay(1000);

		printf("Sweeping down in 1 second...\n");
		delay(1000);
		for(int i=pwm_upper; i>=pwm_lower; i--){
			int val = i;
			printf("PWM Val: %d\n", val);
			pwmWrite(PIN, val);
			delay(5);
		}
		delay(1000);
	}
}
