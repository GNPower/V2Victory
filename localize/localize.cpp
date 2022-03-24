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


inline void get_x_distance(Vehicle_Data *ego, Intersection_Data *target, float* distance){
	distance = (float)(ego->position_x) - (float)(target->position_x);
	}


inline void get_y_distance(Vehicle_Data *ego, Intersection_Data *target, float* distance){
	distance = (float)(ego->position_y) - (float)(target->position_y);
	}


void update_location(Vehicle_Data *ego, uint32_t x_distance, uint32_t y_distance){
	ego->position_x += x_distance;
	ego->position_y += y_distance;
	}


void get_abs_distance(Vehicle_Data *ego, Intersection_Data *target, float* distance){
	float x_distance, y_distance;
	get_x_distance(ego, target, &x_distance);
	get_y_distance(ego, target, &y_distance);
	*distance = sqrt((x_distance*x_distance)+(y_distance*y_distance));
	} 


