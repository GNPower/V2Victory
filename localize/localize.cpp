#include "localize.h"

float get_x_distance_traveled(int enc_value, int heading){
	float cycles = enc_value/20.0;
	const float circumference = 2.0*RADIUS*PI;
	float abs_traveled, x_traveled;

	abs_traveled = circumference*cycles;
	x_traveled = abs_traveled*cos(heading);

	return x_traveled;
	}


float get_y_distance_traveled(int enc_value, int heading){
	float cycles = enc_value/20.0;
	const float circumference = 2.0*RADIUS*PI;
	float abs_traveled, y_traveled;

	abs_traveled = circumference*cycles;
	y_traveled = abs_traveled*sin(heading);

	return y_traveled;
	}


inline float get_x_distance(float current_x, float target_x){
	return (target_x - current_x);
	}


inline float get_y_distance(float current_y, float target_y){
	return (target_y - current_y);
	}


void update_location(Vehicle_Data *ego, uint32_t x_distance, uint32_t y_distance){
	ego->position_x += x_distance;
	ego->position_y += y_distance;
	}


void get_abs_distance(Vehicle_Data *ego, Intersection_Data *target, float* x_y_distance[]){
	//float x_distance = get_x_distance(current_x, target_x);
	//float y_distance = get_x_distance(current_y, target_y);
	//float abs_distance = sqrt((x_distance*x_distance)+(y_distance*y_distance));
	//return abs_distance;
	continue;
	} 


