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

#define L	0
#define R	2
#define BL	5
#define BR	8

// Far ranges = [70,528]
const int pwm_lower = 80;
const int pwm_upper = 510;
const int B_pwm_lower = 150;
const int B_pwm_upper = 200;

static pwm_info pwm_info_t;

int main (int argc, char *argv[]){
	pwm_info_t.ccr = 500;
	pwm_info_t.arr = 1000;
	pwm_info_t.div = 120;
	pwm_info_t.div_stepping = 2;

	wiringPiSetup();
	pinMode(L, PWM_OUTPUT);
	pinMode(R, PWM_OUTPUT);
	pinMode(BL, PWM_OUTPUT);
	pinMode(BR, PWM_OUTPUT);

	pwmSetRange(L, pwm_info_t.arr);
	pwmSetClock(L, pwm_info_t.div);
	pwmWrite(L, pwm_info_t.ccr);

	pwmSetRange(R, pwm_info_t.arr);
	pwmSetClock(R, pwm_info_t.div);
	pwmWrite(R, pwm_info_t.ccr);

	pwmSetRange(BL, pwm_info_t.arr);
	pwmSetClock(BL, pwm_info_t.div);
	pwmWrite(BL, pwm_info_t.ccr);

	pwmSetRange(BR, pwm_info_t.arr);
	pwmSetClock(BR, pwm_info_t.div);
	pwmWrite(BR, pwm_info_t.ccr);

	printf("Zeroing in 1 second...\n");
	delay(1000);
	pwmWrite(L, pwm_lower);
	pwmWrite(R, pwm_lower);
	delay(1000);

	while(true){
		printf("Sweeping up in 1 second...\n");
		delay(1000);
		for(int i=pwm_lower; i<=pwm_upper; i++){
			int val = i;
			printf("PWM Val: %d\n", val);
			pwmWrite(L, val);
			pwmWrite(R, val);
			delay(5);
		}
		delay(1000);

		printf("Sweeping down in 1 second...\n");
		delay(1000);
		for(int i=pwm_upper; i>=pwm_lower; i--){
			int val = i;
			printf("PWM Val: %d\n", val);
			pwmWrite(L, val);
			pwmWrite(R, val);
			delay(5);
		}
		delay(1000);

		printf("Sweeping up in 1 second...\n");
		delay(1000);
		for(int i=B_pwm_lower; i<=B_pwm_upper; i++){
			int val = i;
			printf("PWM Val: %d\n", val);
			pwmWrite(BL, val);
			pwmWrite(BR, val);
			delay(25);
		}
	}
}
