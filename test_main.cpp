#include "stdio.h"

#include "test_top.h"
#include "localize/localize.h"
#include "motor_drive/motor_drive.h"
#include "encoder/encode.h"


int main(){
	Vehicle_Data ego;
	Intersection_Data intersection;

	ego.position_x = 10;
	ego.position_y = 20;

	intersection.position_x = 15;
	intersection.position_y = 25;


	printf("x: %d  y: %d\n", ego.position_x, ego.position_y);
	update_location(&ego, 2, 7);
	printf("x: %d  y: %d\n", ego.position_x, ego.position_y);
	

	float distance;
	get_abs_distance(&ego, &intersection, &distance);
	printf("dist: %l\n", distance);


	printf("No Sleep Till Brooklyn \n");



	return 0;
}