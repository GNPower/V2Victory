#ifndef TOP
#define TOP

#include "stdint.h"

#define TIMESTEP 1000

struct Vehicle_Data{
	uint32_t position_x, position_y;
	uint32_t heading;
	float speed;
	uint8_t priority;
};

typedef Vehicle_Data Vehicle_Data;

struct Intersection_Data{
	uint32_t position_x;
    uint32_t position_y;
    uint8_t num_directions;
    uint8_t directions[4];
    uint8_t intersection_state;
    uint8_t intersection_next_state;
    float intersection_switch_time;
};

typedef Intersection_Data Intersection_Data;



#endif 
