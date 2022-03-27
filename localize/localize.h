//Localization Header for Rapberry Pi - LM298N
//Sam Baker 01/2022

#ifndef LOCAL 
#define LOCAL

#define PI 3.14159

#include <stdio.h>
#include <math.h>

#include "../test_top.h"

void get_x_distance_traveled(Vehicle_Data *ego, float* distance);

void get_y_distance_traveled(Vehicle_Data *ego, float* distance);

void update_location(Vehicle_Data *ego, uint32_t x_distance, uint32_t y_distance);

void get_x_distance(Vehicle_Data *ego, Intersection_Data *target, uint32_t* distance);

void get_y_distance(Vehicle_Data *ego, Intersection_Data *target, uint32_t* distance);

void get_abs_distance(Vehicle_Data *ego, Intersection_Data *target, float* distance);

#endif