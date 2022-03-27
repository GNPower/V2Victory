//Localization Header for Rapberry Pi - LM298N
//Sam Baker 01/2022

#ifndef LOCAL 
#define LOCAL

#define PI 3.14159

#include <stdio.h>
#include <math.h>

#include "../test_top.h"
#include "../encoder/encode.h"


void update_location(Vehicle_Data *ego, uint32_t x_distance, uint32_t y_distance, float timestep);

void get_x_distance(Vehicle_Data *ego, Intersection_Data *target, uint32_t* distance);

void get_y_distance(Vehicle_Data *ego, Intersection_Data *target, uint32_t* distance);

void get_abs_distance(Vehicle_Data *ego, Intersection_Data *target, float* distance);

void get_direction(Vehicle_Data *ego, Intersection_Data *target, float* direction);

#endif