//Encoder Header for Rapberry Pi
//Sam Baker 02/2022


#ifndef ENCODE
#define ENCODE

#define LENCODER 23
#define RENCODER 24

#define RADIUS 3.3
#define PI 3.14159

#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>

#include <poll.h>
#include <string.h>



void* poll_l_encoder(void*);
void* poll_r_encoder(void*);

int get_encoder_value(int encoder);

void get_x_distance_traveled(Vehicle_Data *ego, float* distance);
void get_y_distance_traveled(Vehicle_Data *ego, float* distance);

#endif