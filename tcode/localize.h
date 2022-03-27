//Localization Header for Rapberry Pi - LM298N
//Sam Baker 01/2022

#ifndef LOCAL 
#define LOCAL

#include <stdio.h>
#include <math.h>

#define RADIUS 3.3
#define PI 3.14159


float get_x_distance_traveled(int enc_value, int heading);

float get_y_distance_traveled(int enc_value, int heading);

float update_location(float current, float traveled);

float get_x_distance(float current_x, float target_x);

float get_y_distance(float current_y, float target_y);

float get_abs_distance(float current_x, float current_y, float target_x, float target_y);

#endif