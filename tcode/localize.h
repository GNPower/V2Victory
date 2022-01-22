//Localization Header for Rapberry Pi - LM298N
//Sam Baker 01/2022

#include <stdio.h>
#include <math.h>

#define SLOPE 2.6813
#define OFFSET 17.31


float get_distance_traveled(int duty, int time_ms){
	float time_s = time_ms/1000.0;
	return (duty*SLOPE + OFFSET);
	}


float update_location(float current, float traveled){
	return (current+traveled);
	}


float get_x_distance(float current_x, float target_x){
	return (target_x - current_x);
	}


float get_y_distance(float current_y, float target_y){
	return (target_y - current_y);
	}


float get_abs_distance(float current_x, float current_y, float target_x, float target_y){
	float x_distance = get_x_distance(current_x, target_x);
	float y_distance = get_x_distance(current_y, target_y);
	float abs_distance = sqrt((x_distance*x_distance)+(y_distance*y_distance));
	return abs_distance;
	} 
