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
#include "localize.h"
#include "encode.h"

#include <pthread.h>
#include <signal.h>

#define TIMESTEP 1000

int main(int argc, char *argv[]){
	printf("Don't Stare Directly At The Bugs \n");

	pthread_t left_tid, right_tid;
	int duty_a = atoi(argv[1]);
	int duty_b = atoi(argv[1]);
	int target_x = atoi(argv[2]);
	char* direct = argv[3];

	int stop;
	float location_x = 0;
	float location_y = 0;
	int time = 0;

	int print_counter = 0;
	int encoder_average;

	//Setup GPIO
	if ((-1 == setup_gpio(LFORWARD, OUT))|
		(-1 == setup_gpio(LBACKWARD, OUT))|
		(-1 == setup_gpio(RFORWARD, OUT))|
		(-1 == setup_gpio(STOP, IN))|
		(-1 == setup_gpio(LENCODER, IN))|
		(-1 == setup_gpio(RENCODER, IN))|
		(-1 == setup_gpio(RBACKWARD, OUT)))
		return 2;

	//PWM Setup
	if ((-1 == setup_pwm(ENA, duty_a))|(-1 == setup_pwm(ENB, duty_b)))
		return 2;

	//Setup encoder threads
	pthread_create(&left_tid, NULL, poll_l_encoder, NULL);
	pthread_create(&right_tid, NULL, poll_r_encoder, NULL);

	//Main loop
	while(get_x_distance(location_x, target_x) > 0){
		
		//Choose moter direction
		if (!strcmp(direct, "FORWARD")){
			set_forward();
			//else
				//printf("Moving Forward \n");	
			}

		else if (!strcmp(direct, "BACKWARD")){
			set_backward();
			//else
				//printf("Moving Backward \n");
			}

		else if (!strcmp(direct, "RIGHT")){
			set_right();
			//else
				//printf("Turn Right \n");
			}

		else if (!strcmp(direct, "LEFT")){
			set_left();
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
		encoder_average = (get_encoder_value(LENCODER) + get_encoder_value(RENCODER))/2;
		location_x = update_location(location_x, get_x_distance_traveled(encoder_average, 0));
		time = time+TIMESTEP/1000;

		print_counter++;
		if (print_counter >= 1000){
			print_counter = 0;
			printf("Time: %d, Location: %f, L_ENC: %d, R_ENC %d, ENC AV: %d", time, location_x, get_encoder_value(LENCODER), get_encoder_value(RENCODER), encoder_average);
			}
		}

	//Disable GPIO

	if ((-1 == PWMEnable(ENA, 0))|(-1 == PWMEnable(ENB, 0)))
		return 2;

	if ((-1 == PWMUnexport(ENA))|
		(-1 == PWMUnexport(ENB))|
		(-1 == GPIOUnexport(STOP))|
		(-1 == GPIOUnexport(LENCODER))|
		(-1 == GPIOUnexport(RENCODER))|
		(-1 == GPIOUnexport(LFORWARD))|
		(-1 == GPIOUnexport(LBACKWARD))|
		(-1 == GPIOUnexport(RFORWARD))|
		(-1 == GPIOUnexport(RBACKWARD)))
		return 1;
	
	printf("Aprox Time Moving (ms): %d \n", time);

	//kill encoder threads
	pthread_kill(left_tid, SIGKILL);
	pthread_kill(right_tid, SIGKILL);


	return 0;
}
