//Sam Baker
//V2Victory
//Top level ROS2 vehicle implementation

#ifndef TOP
    #define TOP

///////////////////////////////////////////////INCLUDES/////////////////////////////////////////////////

#include "stdint.h"

//local headers
#include "localize/localize.h"
#include "motor_drive/motor_drive.h"
#include "encoder/encode.h"
#include "vector/vector.hpp"

//c libraries
#include "stdio.h"
#include "time.h"

//c++ libraries
#include <pthread.h>
#include <signal.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

//ROS2 stuff
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "car_interface/msg/car.hpp"
#include "car_interface/msg/intersection.hpp"


///////////////////////////////////////////////Defines/////////////////////////////////////////////////

#define TIMESTEP 100

///////////////////////////////////////////////TYPEDEFS/////////////////////////////////////////////////

//State details for vehicle
struct Vehicle_Data{
	uint32_t position_x, position_y;
	uint32_t heading;
	float speed;
	uint8_t priority;
};

typedef Vehicle_Data Vehicle_Data;


//State details for intersection
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

int Close_All();

#endif 
