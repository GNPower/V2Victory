//defines.hpp
#ifndef SYS_DEFINES
#define SYS_DEFINES

//Defines
//Assumptions
#define MIN_YELLOW 3.7
#define MAX_VEHICLES 32
#define MAX_INTS 3 //this is the amount in range at one time
#define MIN_SET_SPEED 4.4
#define TIMESTEP 0.05 //timestep in s of simulation

//Environment (SIM vs IRL)
#define SIM
#define DEBUG
//#define DRIVER //determines if simulating human driver

//SIM variables
#define INIT_FROM_FILE
//#define ERROR_ENABLE
#define VIS_FILE_ENABLE //write to file for visualization
#define MAX_TIME 40.0 //max time in seconds to stop sim
#define COMMS_RANGE 300.0 //max distance communications can travel
#ifdef ERROR_ENABLE
#define COMMS_ERROR_PCT 1 //% chance of comms failing
#define EGO_DATA_ERROR_PCT 1 //% chance of no ego data
#define ERROR_POS 1.5 //pos error of normal distribution in m (SAE is 1.5, commercial is 2.5) [GPS]
#define ERROR_SPEED 0.5 //speed error of normal distribution in m/s (0.5 is a guess) [IMU] 
#define ERROR_HEADING 3.0 //heading error of normal distribution in degs (3.0 is from paper) [GPS]
#define ERROR_ACCEL 0.1 //accel standard error in m/s^2 (0.1 is a guess)
#else
#define COMMS_ERROR_PCT 0 //% chance of comms failing
#define EGO_DATA_ERROR_PCT 0 //% chance of no ego data
#define ERROR_POS 0 //pos error of normal distribution in m (SAE is 1.5, commercial is 2.5) [GPS]
#define ERROR_SPEED 0 //speed error of normal distribution in m/s (0.5 is a guess) [IMU] 
#define ERROR_HEADING 0 //heading error of normal distribution in degs (3.0 is from paper) [GPS]
#define ERROR_ACCEL 0 //accel standard error in m/s^2 (0.1 is a guess)
#endif

//Driver Tunable
#define DRIVER_REACTION 0.25 //time in seconds for driver to react to visual changes

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