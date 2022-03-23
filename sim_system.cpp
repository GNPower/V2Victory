#include "defines.hpp"
#include "common.hpp"
#include "vehicle.hpp"
#include "intersection.hpp"
#include <iostream>

int main(void)
{
    //Initialize Intersection
    double ent_dirs[2] = {0.0, 90.0};
    double tts[3] = {30.0, 4.0, 40.0};
    double int_length = 10.0;
    double int_pos_x = 0.0;
    double int_pos_y = 0.0;
    y_int_type int_type = SIGNAL;
    unsigned int int_id = 1;
    double ped_countdown = 5.0;

    std::cout << "Initializing Intersection...\n";
    intersection test_int(ent_dirs, int_length, int_pos_x, int_pos_y, int_type, int_id, ped_countdown, tts);
    std::cout << "Intersection Initialized! \n";

    //Initialize Vehicle
    unsigned int veh_id = 2;
    double max_speed = 50.0;
    y_veh_type veh_type = NORMAL;
    double front_pos = 3.0;
    double set_speed = 20.0;
    double pos_x = -10.0;
    double pos_y = 0.0;
    double speed = 0.0;
    double heading = 0.0;
    bool override = false;
    bool platooning = false;
    double driver_accel = 0.0;

    std::cout << "Initializing Vehicles...\n";
    vehicle test_vehicle(veh_id, max_speed, veh_type, front_pos, set_speed, pos_x, pos_y, speed, heading, override, platooning, driver_accel);
    std::cout << "Vehicles Initialized! \n";
}