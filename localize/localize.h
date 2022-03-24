//Localization Header for Rapberry Pi - LM298N
//Sam Baker 01/2022

#ifndef LOCAL 
#define LOCAL

#include <stdio.h>
#include <math.h>

#include "../test_top.h"

#define RADIUS 3.3
#define PI 3.14159


float get_x_distance_traveled(int enc_value, int heading);

float get_y_distance_traveled(int enc_value, int heading);

void update_location(Vehicle_Data *ego, uint32_t x_distance, uint32_t y_distance);

float get_x_distance(float current_x, float target_x);

float get_y_distance(float current_y, float target_y);

void get_abs_distance(Vehicle_Data *ego, Intersection_Data *target, float* x_y_distance[]);

#endif