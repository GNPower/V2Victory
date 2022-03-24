#ifndef TOP
#define TOP

#include "stdint.h"

#include "localize/localize.h"
#include "motor_drive/motor_drive.h"
#include "encoder/encode.h"


struct Vehicle_Data{
	uint32_t position_x, position_y;
	uint32_t heading;
	uint32_t speed;
	uint8_t priority;
};

typedef Vehicle_Data Vehicle_Data;

struct Intersection_Data{
	uint32_t position_x, position_y;
	uint32_t Directions[4];
	uint8_t State;
	uint8_t Next_State;
	uint32_t Next_State_Change;
};

typedef Intersection_Data Intersection_Data;



#endif 
