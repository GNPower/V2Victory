#include "localize.h"





inline void get_x_distance(Vehicle_Data *ego, Intersection_Data *target, float* distance){
	*distance = (float)ego->position_x - (float)target->position_x;
	}


inline void get_y_distance(Vehicle_Data *ego, Intersection_Data *target, float* distance){
	*distance = (float)ego->position_y - (float)target->position_y;
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

void get_direction(Vehicle_Data *ego, Intersection_Data *target, float* direction){
	float x_distance, y_distance;
	get_x_distance(ego, target, &x_distance);
	get_y_distance(ego, target, &y_distance);
	*direction = (atan2(y_distance, x_distance)*360)/(2*PI);
	} 

void get_speed(Vehicle_Data *ego, float timestep){
	float x_distance, y_distance, abs_distance;
	get_x_distance_traveled(&ego, &x_distance);
	get_y_distance_traveled(&ego, &y_distance);
	abs_distance = sqrt(x_distance*x_distance + y_distance*y_distance);
	ego->speed = (uint32_t)(abs_distance/timestep);
}
