//Motor Driver Test Script for Rapberry Pi - LM298N
//Sam Baker 01/2022
//
//
//Accepts command line args
//        - int duty to set speed
//        - int target distance in cm
//        - string [FORWARD, BACKWARD, LEFT, RIGHT] direction


#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#include "motor_drive.h"


#define TIMESTEP 1000

#define LIGHT_DISTANCE 400
#define RED 1
#define GREEN 0

int main(int argc, char *argv[]){
	printf("Onlookers were shocked \n");
	
	int duty = atoi(argv[1]);
	int target_x = LIGHT_DISTANCE;
	int light_state = RED;
	int count = 0;
	int stop;
	float location_x = 0;
	float location_y = 0;
	int time = 0;

	//Enable GPIOs
	if ((-1 == PWMExport(ENA))|
		(-1 == PWMExport(ENB))|
		(-1 == GPIOExport(LFORWARD))|
		(-1 == GPIOExport(LBACKWARD))|
		(-1 == GPIOExport(RFORWARD))|
		(-1 == GPIOExport(RBACKWARD)))
		return 1;


	if (-1 == GPIOExport(STOP))
		return 1;

	//Set Direction
	if (/*(-1 == GPIODirection(ENA, OUT))|
		(-1 == GPIODirection(ENB, OUT))|*/
		(-1 == GPIODirection(LFORWARD, OUT))|
		(-1 == GPIODirection(LBACKWARD, OUT))|
		(-1 == GPIODirection(RFORWARD, OUT))|
		(-1 == GPIODirection(RBACKWARD, OUT)))
		return 2;

	if (-1 == GPIODirection(STOP, IN))
		return 2;

	//PWM Setup
	//PWM Setup
	if ((-1 == PWMPeriod(ENA))|(-1 == PWMPeriod(ENB)))
		return 2;

	if ((-1 == PWMDuty(ENA, duty))|(-1 == PWMDuty(ENB, duty)))
		return 2;

	if ((-1 == PWMEnable(ENA, 1))|(-1 == PWMEnable(ENB, 1)))
		return 2;


	//Main loop
	while(1){
		set_forward();

		//E-Stop
		if(GPIORead(STOP)) break;

		//Wait and Update
		usleep(TIMESTEP);
		location_x = update_location(location_x, get_distance_traveled(duty, TIMESTEP/1000));
		time = time+TIMESTEP/1000;

		count++;
		if (count == 100){ 

			printf("Location: %f  Light Loc: %d   D2L: %f    LS:   %d \n", 
					location_x, target_x, get_x_distance(location_x, target_x), light_state);
			count = 0;
		}

		if ((get_x_distance(location_x, target_x) < 200) && (light_state == RED)){
			
			if (count == 50){
				duty = duty - 4;

				if (-1 == PWMDuty(ENA, duty))
					return 2;
				
				printf("LIGHT REACHED: RED    Duty %d \n", duty);
				}
			}

		if (duty < 0) break; 
		}

	//Disable GPIO

	if ((-1 == PWMEnable(ENA, 0))|(-1 == PWMEnable(ENB, 0)))
		return 2;


	if ((-1 == PWMUnexport(ENA))|(-1 == PWMUnexport(ENB)))
		return 1;

	if ((-1 == GPIOUnexport(LFORWARD))|
		(-1 == GPIOUnexport(LBACKWARD))|
		(-1 == GPIOUnexport(RFORWARD))|
		(-1 == GPIOUnexport(RBACKWARD)))
		return 1;
	
	printf("Aprox Time Moving (ms): %d \n", time);
	return 0;
}
