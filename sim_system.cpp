#include "defines.hpp"
#include "common.hpp"
#include "vehicle.hpp"
#include "intersection.hpp"
#include <iostream>
#include <vector>
#include <random>

#define ERROR_ENABLE
#define TIMESTEP 0.5 //timestep in s of simulation
#define MAX_TIME 10.0 //max time in seconds to stop sim

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

//Enum Arrays
const char *int_states[3] = {"GREEN", "YELLOW", "RED"};

//Structs
struct s_veh_sim_data{
    double pos_x;
    double pos_y;
    double speed;
    double heading;
    double accel_req;
};

//setup errors
std::default_random_engine generator;
std::normal_distribution<double> pos_error(0, ERROR_POS);
std::normal_distribution<double> speed_error(0, ERROR_SPEED);
std::normal_distribution<double> heading_error(0, ERROR_HEADING);
std::normal_distribution<double> accel_error(0, ERROR_ACCEL);

s_veh_sim_data update_veh_data(s_veh_sim_data input_veh)
{
    //updates actual/ground truth pos_x, pos_y, speed -> speed is not perfect match of accel_req
    s_veh_sim_data output_veh;
    output_veh.accel_req = input_veh.accel_req;
    output_veh.heading = input_veh.heading;
    output_veh.speed = fmax(input_veh.speed + TIMESTEP*(input_veh.accel_req + accel_error(generator)), 0); //fmax ensures speed isnt negative since we use heading
    output_veh.pos_x = input_veh.pos_x + TIMESTEP*(input_veh.speed + output_veh.speed)/2*cos(M_PI/180*output_veh.heading);
    output_veh.pos_y = input_veh.pos_y + TIMESTEP*(input_veh.speed + output_veh.speed)/2*sin(M_PI/180*output_veh.heading);
    
    return output_veh;
}

s_veh_sim_data add_veh_error(s_veh_sim_data input_veh)
{
    //updates pos_x, pos_y, speed, heading read by vehicle
    s_veh_sim_data output_veh;
    output_veh.accel_req = input_veh.accel_req;
    output_veh.pos_x = input_veh.pos_x + pos_error(generator);
    output_veh.pos_y = input_veh.pos_y + pos_error(generator);
    output_veh.speed = input_veh.speed + speed_error(generator);
    output_veh.heading = input_veh.heading + heading_error(generator);

    return output_veh;
}

void print_sim_int_data(s_published_int_data input_int)
{
    std::cout << "\nINTERSECTION\n";
    std::cout << "Position X: " << input_int.int_pos_x << "\tPosition Y: " << input_int.int_pos_y << "\n";
    std::cout << "Dir #1: " << input_int.ent_dir_1 << "\tDir #2: " << input_int.ent_dir_2 << "\n";

    if (input_int.int_type == SIGNAL)
    {
        std::cout << "{Dir #1} State: " << int_states[input_int.int_1_state] << "\tNext State: " << int_states[input_int.int_1_next_state] << "\tTTS: " << input_int.int_1_time_to_switch << "\n";
        std::cout << "{Dir #2} State: " << int_states[input_int.int_2_state] << "\tNext State: " << int_states[input_int.int_2_next_state] << "\tTTS: " << input_int.int_2_time_to_switch << "\n";
    }
    else
    {
        std::cout << "Next Vehicle ID: " << input_int.int_next_veh_id << "\n";
    }
}

void print_sim_veh_data(s_veh_sim_data input_veh)
{
    std::cout << "\nVEHICLE\n";
    std::cout << "Position X: " << input_veh.pos_x << "\tPosition Y: " << input_veh.pos_y << "\n";
    std::cout << "Speed: " << input_veh.speed << "\tHeading: " << input_veh.heading << "\n";
    std::cout << "Accel Req: " << input_veh.accel_req << "\n";
}

int main(void)
{
    unsigned int id = 1;
    //Initialize Intersection(s)
    int num_ints = 1;
    std::vector<intersection> int_list;
    s_published_int_data int_published[num_ints];

    for (int i = 0; i < num_ints; i++)
    {
        double ent_dirs[2] = {0.0, 90.0};
        double tts[3] = {30.0, 4.0, 40.0};
        double int_length = 10.0;
        double int_pos_x = 0.0;
        double int_pos_y = 0.0;
        y_int_type int_type = SIGNAL;
        unsigned int int_id = id;
        id++;
        double ped_countdown = 5.0;

        std::cout << "Initializing Intersection...\n";
        int_list.push_back(intersection (ent_dirs, int_length, int_pos_x, int_pos_y, int_type, int_id, ped_countdown, tts));
        std::cout << "Intersection Initialized! \n";
    }
    std::cout << "All Intersections Initialized! \n";

    //Initialize Vehicle(s)
    int num_vehs = 1;
    std::vector<vehicle> veh_list;
    s_published_vehicle_data veh_published[num_vehs];
    //need to initialize the actual dynamics of vehicle
    s_veh_sim_data sim_vehs[num_vehs];

    for (int i = 0; i < num_vehs; i++)
    {
        unsigned int veh_id = id;
        id++;
        double max_speed = 50.0;
        y_veh_type veh_type = NORMAL;
        double front_pos = 3.0;
        double set_speed = 20.0;
        double pos_x = 0.0;
        double pos_y = -10.0;
        double speed = 0.0;
        double heading = 90.0;
        bool override = false;
        bool platooning = false;
        double driver_accel = 0.0;

        std::cout << "Initializing Vehicle...\n";
        veh_list.push_back(vehicle (veh_id, max_speed, veh_type, front_pos, set_speed, pos_x, pos_y, speed, heading, override, platooning, driver_accel));
        sim_vehs[i].pos_x = pos_x;
        sim_vehs[i].pos_y = pos_y;
        sim_vehs[i].speed = speed;
        sim_vehs[i].heading = heading;
        sim_vehs[i].accel_req = veh_list[i].get_accel_req();
        std::cout << "Vehicle Initialized! \n";
    }
    std::cout << "All Vehicles Initialized! \n";

    //Setup Simulation
    bool sim_done = false;
    double sim_time = 0;

    while (!sim_done)
    {
        std::cout << "\nTime: " << sim_time;
        
        //collect int data
        for (int i = 0; i < num_ints; i++)
        {
            int_published[i] = int_list[i].publish();
            print_sim_int_data(int_published[i]);
        }

        //collect veh data
        for (int i = 0; i < num_vehs; i++)
        {
            veh_published[i] = veh_list[i].publish();
        }

        int comms_pct;
        //send data to eachother
        for (int i = 0; i < num_ints; i++)
        {
            for (int j = 0; j < num_vehs; j++)
            {
                //send veh data to ints
                comms_pct = (rand() % 100) + 1;
                if (comms_pct > COMMS_ERROR_PCT)
                {
                    //comms work
                    int_list[i].new_vehicle_callback(veh_published[j]);
                }

                //send int data to vehs
                comms_pct = (rand() % 100) + 1;
                if (comms_pct > COMMS_ERROR_PCT)
                {
                    //comms work
                    veh_list[j].new_int_callback(int_published[i]);
                }
            }
        }
        //send data between vehicles
        for (int i = 0; i < num_vehs; i++)
        {
            for (int j = 0; j < num_vehs; j++)
            {
                if (i == j)
                {
                    //same vehicle, skip
                    continue;
                }

                comms_pct = (rand() % 100) + 1;
                if (comms_pct > COMMS_ERROR_PCT)
                {
                    //comms work
                    veh_list[i].new_vehicle_callback(veh_published[j]);
                }
            }
        }

        //now that data has been published and callbacks have run, can update
        //update ints
        for (int i = 0; i < num_ints; i++)
        {
            int_list[i].update(TIMESTEP);
        }

        //update vehs
        for (int i = 0; i < num_vehs; i++)
        {
            //first update pos, speed based on accel req (with some error)
            sim_vehs[i] = update_veh_data(sim_vehs[i]);
            
            //now add error to measured pos, speed, heading (actual values are unchanged)
            s_veh_sim_data veh_error_data = add_veh_error(sim_vehs[i]);

            //publish vehicle's sensors
            std::cout << "\nVEHICLE FEEDBACK\n";
            print_sim_veh_data(veh_error_data);

            //update dynamics (maybe)
            int ego_data_pct = (rand() % 100) + 1;
            if (ego_data_pct > EGO_DATA_ERROR_PCT)
            {
                //ego data works
                veh_list[i].ego_data_callback(veh_error_data.pos_x, veh_error_data.pos_y, veh_error_data.speed, veh_error_data.heading);
            }

            //now update vehicle
            veh_list[i].update(TIMESTEP);

            //get accel req
            sim_vehs[i].accel_req = veh_list[i].get_accel_req();

            print_sim_veh_data(sim_vehs[i]);

        }

    sim_time += TIMESTEP;
    if (sim_time > MAX_TIME)
    {
        sim_done = true;
    }
    }
    std::cout << "Simulation Ended!\n";
}