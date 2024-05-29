#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <stdbool.h>

#include "servo.h"
#include "brushless.h"


int main(){
    servo Servo_L;
    servo Servo_R;
    Servo_L.servo_setup(0);
    Servo_R.servo_setup(2);
    Servo_L.servo_angle(0);
    Servo_R.servo_angle(180);
    brushless Brushless_L;
    brushless Brushless_R;
    Brushless_L.brushless_setup(5);
    Brushless_R.brushless_setup(16);
    Brushless_L.brushless_thrust(0);
    Brushless_R.brushless_thrust(0);
    delay(5000);
    while(1){
        printf("Sweeping up in 1 second...\n");
		delay(1000);
		for(int i=0; i<=180; i++){
			int val = i;
			printf("Servo angle: %d\n degrees", val);
			Servo_L.servo_angle(i);
            Servo_R.servo_angle(180 - i);
			delay(5);
        }
        printf("Sweeping down in 1 second...\n");
		delay(1000);
		for(int i=0; i<=180; i++){
			int val = i;
			printf("Servo angle: %d\n degrees", val);
			Servo_L.servo_angle(180 - i);
            Servo_R.servo_angle(i);
			delay(5);
        }
        delay(3000);
        printf("Forward thrust in 1 second...\n");
		delay(1000);
		for(int i=0; i<=70; i++){
			int val = i;
			printf("Brushless thrust: %d\n", val);
			Brushless_L.brushless_thrust(i);
            Brushless_R.brushless_thrust(i);
			delay(10);
        }
        delay(2000);
        Brushless_L.brushless_thrust(0);
        Brushless_R.brushless_thrust(0);
        printf("Backward thrust in 1 second...\n");
		delay(1000);
		for(int i=0; i>=-70; i--){
			int val = i;
			printf("Brushless thrust: %d\n", val);
			Brushless_L.brushless_thrust(i);
            Brushless_R.brushless_thrust(i);
			delay(10);
        }
        delay(2000);
        Brushless_L.brushless_thrust(0);
        Brushless_R.brushless_thrust(0);
        delay(3000);
    }
    return(1);
}