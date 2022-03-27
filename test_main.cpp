#include "stdio.h"

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
	if ((-1 == setup_gpio(LFORWARD, OUT))|
		(-1 == setup_gpio(LBACKWARD, OUT))|
		(-1 == setup_gpio(RFORWARD, OUT))|
		(-1 == setup_gpio(STOP, IN))|
		(-1 == setup_gpio(LENCODER, IN))|
		(-1 == setup_gpio(RENCODER, IN))|
		(-1 == setup_gpio(RBACKWARD, OUT)))
		return 2;

	if (-1 == GPIOEdge(LENCODER, FALLING))
		return 2;
	if (-1 == GPIOEdge(RENCODER, FALLING))
		return 2;

	//PWM Setup
	if ((-1 == setup_pwm(ENA, duty_a))|
		(-1 == setup_pwm(ENB, duty_b)))
		return 2;
	/////////////////////////////////////////////////////////////////////////////////////////////////
	init_encoders(&left_tid, &right_tid);
	////////////////////////////////////////////////////////////////////////////////////////////////

	Vehicle_Data ego;
	Intersection_Data intersection;

	ego.position_x = 10;
	ego.position_y = 20;

	intersection.position_x = 15;
	intersection.position_y = 25;


	printf("x: %d  y: %d\n", ego.position_x, ego.position_y);
	update_location(&ego, 2, 7);
	printf("x: %d  y: %d\n", ego.position_x, ego.position_y);
	
	init_encoders(&left_tid, &right_tid);

	float distance;
	get_abs_distance(&ego, &intersection, &distance);
	printf("dist: %f\n", distance);

	

	printf("No Sleep Till Brooklyn \n");

	while(1){
		get_x_distance_traveled(&ego, &distance_x);
		printf("X_t: %d\t", distance_x);
		get_y_distance_traveled(&ego, &distance_y);
		printf("Y_t: %d\t", distance_y);
		update_location(&ego, distance_x, distance_y);
		printf("X: %d, Y: %d \n", ego.position_x, ego.position_y);

		usleep(100000);
	}

	pthread_kill(left_tid, SIGKILL);
	pthread_kill(right_tid, SIGKILL);



	return 0;
}