//defines.hpp
#ifndef SYS_DEFINES
#define SYS_DEFINES

//Defines
//Assumptions
#define MIN_YELLOW 3.7
#define MAX_VEHICLES 32
#define MAX_INTS 3 //this is the amount in range at one time
#define MIN_SET_SPEED 4.4 

//Environment (SIM vs IRL)
#define SIM

//Performance/Tunable
//Intersection & Vehicle Data
#define MAX_COASTING 1.0 //amount of time a vehicle or intersection can be coasted before removed (coasted means no state updates received)

//Vehicle Intersection Entrance Assignment
#define INLANE_HEADING 45.0 //angle diff between lane and vehicle that counts as being in same direction

//Dynamic Light Timing Prioritization
#define TRAFFIC_ABS_DIFF 10 //amount of cars difference to trigger switch
#define TRAFFIC_REL_DIFF 4.0 //relative difference in amount of cars to trigger switch

//Stop Sign Intersection
#define MAX_STOP_SIGN_COAST 0.1 //how long since last update at stop sign
#define STOPPED_SPEED 0.1 //minimum speed considered stop (tolerance)
#define STOPPED_TIME 0.5 //amount of time vehicle must stop at stop sign before continuing
#define STOP_SIGN_DISTANCE 2.0 //range vehicle must be from entrance to be considered stopped at the intersection

//Intersection EMS Data
#define EMS_COASTING 0.1 //how long since last update for EMS override

//Vehicle Accel Limits
#define MAX_ACCEL 4.0 //maximum accel of vehicle in m/s^2
#define MAX_DECEL 4.9 //maximum decel of vehicle in m/s^2

//Vehicle Lead Finding
#define INPATH_WIDTH 2.5 //distance to either side that is considered inpath

//Vehicle Intersection Assumptions
#define MIN_RED_GREEN_TIME 10.0 //minimum amount of time assumed for red/green light

//Vehicle Lead Gap
#define LEAD_GAP_DIST 3.0 //min distance in meters from lead
#define LEAD_GAP_TIME 1.5 //min distance in seconds from lead
#define PLATOONING_GAP 15.0 //distance in meters for platooning

//Vehicle Intersection Safety Factor
#define TTS_SAFETY_FACTOR 0.5 //seconds to help ensure not entering on red

//Stretch Goals
#define SMART_TIMING_ENABLE //Stretch Goal #1: Smart Signal Timing
#define EMS_OVERRIDE_ENABLE //Stretch Goal #2: Emergency Vehicle Response
#define STOP_SIGN_ENABLE //Stretch Goal #4: Stop Sign Intersections
#define PLATOONING_ENABLE //Stretch Goal #5: Platooning System

#endif