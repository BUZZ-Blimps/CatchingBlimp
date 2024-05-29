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


	// PREVIOUS LIMITS FOUND: [70, 528]

	printf("Lower Range\n");
	printf("200\n");

	for(int i=60; i<=80; i+=1){
		printf("\t%d", i);
		fflush(stdout);
		pwmWrite(PIN, i);
		delay(500);
		printf(" -> 200\n");
		pwmWrite(PIN, 200);
		delay(500);
	}
	printf("\n");

	printf("Upper Range\n");
	printf("480\n");

	for(int i=535; i>=525; i-=1){
		printf("\t%d", i);
		fflush(stdout);
		pwmWrite(PIN, i);
		delay(500);
		printf(" -> 480\n");
		pwmWrite(PIN, 480);
		delay(500);
	}
	printf("\n");
}
