#include "localize.h"





inline void get_x_distance(Vehicle_Data *ego, Intersection_Data *target, float* distance){
	*distance = (float)ego->position_x - (float)target->position_x;
	}


inline void get_y_distance(Vehicle_Data *ego, Intersection_Data *target, float* distance){
	*distance = (float)ego->position_y - (float)target->position_y;
	}


void update_location(Vehicle_Data *ego, uint32_t x_distance, uint32_t y_distance, float timestep){
	static float distance_buffer[BUFFER_SIZE];
	distance_buffer[0] = sqrt(x_distance*x_distance + y_distance*y_distance);
	float speed_unconverted = 0;

	for (int i = 0; i < BUFFER_SIZE; i++){
		speed_unconverted += distance_buffer[i];
	}

	speed_unconverted = speed_unconverted/(timestep*(BUFFER_SIZE-1));
	ego->speed = (uint32_t)(speed_unconverted*0.001);

	for (int i = BUFFER_SIZE-1; i > 0; i--){
		distance_buffer[i] = distance_buffer[i-1];
	}

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

