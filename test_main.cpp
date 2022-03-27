#include "stdio.h"
#include "time.h"

#include "test_top.h"
#include "localize/localize.h"
#include "motor_drive/motor_drive.h"
#include "encoder/encode.h"

#include <pthread.h>
#include <signal.h>



int main(int argc, char *argv[]){
	pthread_t left_tid, right_tid;
	float distance_x, distance_y;
	int duty_a = atoi(argv[1]);
	int duty_b = atoi(argv[1]);



	//Setup GPIOS///////////////////////////////////////////////////////////////////////////////////
	if (2 == GPIO_init(duty_a, duty_b)){
		printf("Error Initiating GPIOs");
	}
	
	init_encoders(&left_tid, &right_tid);
	
	clock_t past = clock();
	////////////////////////////////////////////////////////////////////////////////////////////////

	Vehicle_Data ego;
	Intersection_Data intersection;

	ego.position_x = 0;
	ego.position_y = 0;
	ego.heading = atoi(argv[2]);

	intersection.position_x = 100;
	intersection.position_y = 200;


	///////////////////////////////////////////////////////////////////////////////////////////

	/*
	printf("x: %d  y: %d\n", ego.position_x, ego.position_y);
	update_location(&ego, 2, 7);
	printf("x: %d  y: %d\n", ego.position_x, ego.position_y);

	float distance;
	get_abs_distance(&ego, &intersection, &distance);
	printf("dist: %f\n", distance);
	*/
	

	printf("No Sleep Till Brooklyn \n");

	int count = 0;
	clock_t past_time = clock();
	clock_t current_time = clock();
	double time_passed;
	while(1){

		current_time = clock();
		time_passed = (double)(current_time - past_time)/CLOCKS_PER_SECOND;
		printf("TIME: %f \t", time_passed);
		past_time = current_time;

		
		get_x_distance_traveled(&ego, &distance_x);
		//printf("X_t: %f\t", distance_x);
		get_y_distance_traveled(&ego, &distance_y);
		//printf("Y_t: %f\t", distance_y);
		update_location(&ego, distance_x, distance_y);
		

		if (count > 100){
			count = 0;
			printf("X: %d, Y: %d \n", ego.position_x, ego.position_y);
		}

		count++;
		usleep(TIMESTEP);


	}

	pthread_kill(left_tid, SIGKILL);
	pthread_kill(right_tid, SIGKILL);



	return 0;
}