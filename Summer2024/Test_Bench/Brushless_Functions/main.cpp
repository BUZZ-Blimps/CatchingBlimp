#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <stdbool.h>

#include "brushless.h"


int main(){
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
		for(int i=0; i<=70; i++){
			int val = i;
			printf("Brushless thrust: %d\n", val);
			Brushless_L.brushless_thrust(i);
            Brushless_R.brushless_thrust(i);
			delay(5);
        }
        delay(2000);
        Brushless_L.brushless_thrust(0);
        Brushless_R.brushless_thrust(0);
        printf("Sweeping down in 1 second...\n");
		delay(1000);
		for(int i=0; i>=-70; i--){
			int val = i;
			printf("Brushless thrust: %d\n", val);
			Brushless_L.brushless_thrust(i);
            Brushless_R.brushless_thrust(i);
			delay(5);
        }
        delay(2000);
        Brushless_L.brushless_thrust(0);
        Brushless_R.brushless_thrust(0);
        delay(3000);
    }
    return(1);
}