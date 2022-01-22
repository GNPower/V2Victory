//Motor Driver for Rapberry Pi - LM298N
//Sam Baker 01/2022
//
//
//Accepts command line arg int from 10-25 to set speed


#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#include "motor_drive.h"


int main(int argc, char *argv[]){
	printf("Onlookers were shocked \n");
	int duty = atoi(argv[1]);
	int time_ms = atoi(argv[2]);
	int stop;
	char* direct = argv[3];

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
	while(time_ms > 0){
		
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
			printf("Problem with instructions\n Please include command line args for: \n int speed, int time and str direct {FORWARD, BACKWARD, RIGHT, LEFT}");
		}

	if(GPIORead(STOP)) break;

	//speedSet(duty);
	time_ms = time_ms - 20;
	usleep(20*1000);
	//toggle = !toggle;
	
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
	
	
	return 0;
}
