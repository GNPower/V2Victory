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


#define TIMESTEP 2000

int main(int argc, char *argv[]){
	printf("Onlookers were shocked \n");
	
	int duty = atoi(argv[1]);
	int target_x = atoi(argv[2]);
	char* direct = argv[3];

	int stop;
	float location_x = 0;
	float location_y = 0;
	int time = 0;

	//Enable GPIOs
	if ((-1 == PWMExport(ENA))|
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
	if (-1 == PWMPeriod(ENA))
		return 2;

	if (-1 == PWMDuty(ENA, duty))
		return 2;

	if (-1 == PWMEnable(ENA, 1))
		return 2;


	//Main loop
	while(get_x_distance(location_x, target_x) > 0){
		
		//Choose moter direction
		if (!strcmp(direct, "FORWARD")){
			if ((-1 == GPIOWrite(LFORWARD, 1))|
				(-1 == GPIOWrite(LBACKWARD, 0))|
				(-1 == GPIOWrite(RBACKWARD, 0))|
				(-1 == GPIOWrite(RFORWARD, 1)))
				return 3;
			//else
				//printf("Moving Forward \n");	
			}

		else if (!strcmp(direct, "BACKWARD")){
			if ((-1 == GPIOWrite(LFORWARD, 0))|
				(-1 == GPIOWrite(LBACKWARD, 1))|
				(-1 == GPIOWrite(RBACKWARD, 1))|
				(-1 == GPIOWrite(RFORWARD, 0)))
				return 3;
			//else
				//printf("Moving Backward \n");
			}

		else if (!strcmp(direct, "RIGHT")){
			if ((-1 == GPIOWrite(LFORWARD, 1))|
				(-1 == GPIOWrite(LBACKWARD, 0))|
				(-1 == GPIOWrite(RBACKWARD, 1))|
				(-1 == GPIOWrite(RFORWARD, 0)))
				return 3;
			//else
				//printf("Turn Right \n");
			}

		else if (!strcmp(direct, "LEFT")){
			if ((-1 == GPIOWrite(LFORWARD, 0))|
				(-1 == GPIOWrite(LBACKWARD, 1))|
				(-1 == GPIOWrite(RBACKWARD, 0))|
				(-1 == GPIOWrite(RFORWARD, 1)))
				return 3;
			//else
				//printf("Turn Left \n");
			}

		else{
			printf("Problem with instructions\n Please include command line args for: \n int speed, int distance and str direct {FORWARD, BACKWARD, RIGHT, LEFT}");
			}

		//E-Stop
		if(GPIORead(STOP)) break;

		//Wait and Update
		usleep(TIMESTEP);
		location_x = update_location(location_x, get_distance_traveled(duty, TIMESTEP/1000));
		time = time+TIMESTEP/1000;
		}

	//Disable GPIO
	if (-1 == PWMEnable(ENA, 0))
		return 2;

	if ((-1 == PWMUnexport(ENA))|
		(-1 == GPIOUnexport(LFORWARD))|
		(-1 == GPIOUnexport(LBACKWARD))|
		(-1 == GPIOUnexport(RFORWARD))|
		(-1 == GPIOUnexport(RBACKWARD)))
		return 1;
	
	printf("Aprox Time Moving (ms): %d \n", time);
	return 0;
}
