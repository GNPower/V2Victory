#include "defines.hpp"
#include "common.hpp"
#include "vehicle.hpp"
#include "intersection.hpp"
#include <iostream>
#include <vector>
#include <random>
#include <fstream>
#include <iomanip>

//#define ERROR_ENABLE
#define VIS_FILE_ENABLE //write to file for visualization
#define TIMESTEP 0.5 //timestep in s of simulation
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

//setup file
#ifdef VIS_FILE_ENABLE
std::ofstream vis_file;
#endif

//setup errors
std::default_random_engine generator;
std::normal_distribution<double> pos_error(0, ERROR_POS);
std::normal_distribution<double> speed_error(0, ERROR_SPEED);
std::normal_distribution<double> heading_error(0, ERROR_HEADING);
std::normal_distribution<double> accel_error(0, ERROR_ACCEL);

bool comms_in_range(double node1_x, double node1_y, double node2_x, double node2_y)
{
    double dist_x = node2_x - node1_x;
    double dist_y = node2_y - node1_y;
    double dist = sqrt(pow(dist_x, 2) + pow(dist_y, 2));
    
    bool in_range = dist <= COMMS_RANGE;
    return in_range;
}

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
    output_veh.speed = fmax(input_veh.speed + speed_error(generator), 0); //scalar
    output_veh.heading = input_veh.heading + heading_error(generator);
    while (output_veh.heading >= 360)
    {
        output_veh.heading -= 360;
    }
    while (output_veh.heading < 0)
    {
        output_veh.heading += 360;
    }
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

#ifdef VIS_FILE_ENABLE
void write_sim_int_data(s_published_int_data input_int)
{
    vis_file << ";(" << int_states[input_int.int_1_state][0] << "," << int_states[input_int.int_2_state][0] << ")";
}
#endif

void print_sim_veh_data(s_veh_sim_data input_veh, bool actual_data, unsigned int id) //actual_data: true = sim, false = feedback (w/error)
{
    if (actual_data)
    {
        std::cout << "\nVEHICLE ACTUAL DATA\n";
    }
    else
    {
        std::cout << "\nVEHICLE FEEDBACK DATA\n";
    }
    
    std::cout << "Position X: " << input_veh.pos_x << "\tPosition Y: " << input_veh.pos_y << "\n";
    std::cout << "Speed: " << input_veh.speed << "\tHeading: " << input_veh.heading << "\n";
    std::cout << "Accel Req: " << input_veh.accel_req << "\n";
}

#ifdef VIS_FILE_ENABLE
void write_sim_veh_data(s_veh_sim_data input_veh, unsigned int id)
{
    vis_file << std::fixed << ";(" << id << "," << std::setprecision(2) << input_veh.pos_x << "," << std::setprecision(2) << input_veh.pos_y << "," << std::setprecision(2) << input_veh.heading << ")"; 
}
#endif

int main(void)
{
    //Setup Simulation
    bool sim_done = false;
    double sim_time = 0;
    std::cout << "Time: " << sim_time << "\n";

    //open file if needed
    #ifdef VIS_FILE_ENABLE
    vis_file.open("vis.log");
    vis_file << "(0.00";
    #endif

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
        double ped_countdown = 1.0;

        std::cout << "Initializing Intersection...\n";
        int_list.push_back(intersection (ent_dirs, int_length, int_pos_x, int_pos_y, int_type, int_id, ped_countdown, tts));
        std::cout << "Intersection Initialized! \n";

        #ifdef VIS_FILE_ENABLE
        write_sim_int_data(int_list[i].publish());
        #endif
    }
    std::cout << "All Intersections Initialized! \n";

    //Initialize Vehicle(s)
    int num_vehs = 2;
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
        double front_pos = 0.0;
        double set_speed = 20.0;
        double pos_x = 0.0;
        double pos_y = -50.0 - double(30*i);
        double speed = 0.0;
        double heading = 90.0;
        bool override = false;
        bool platooning = false;
        double driver_accel = 0.0;

        std::cout << "Initializing Vehicle...\n";
        sim_vehs[i].pos_x = pos_x;
        sim_vehs[i].pos_y = pos_y;
        sim_vehs[i].speed = speed;
        sim_vehs[i].heading = heading;
        sim_vehs[i].accel_req = 0;
        print_sim_veh_data(sim_vehs[i], true, veh_id); //t = sim_time
        s_veh_sim_data veh_error_data = add_veh_error(sim_vehs[i]);
        print_sim_veh_data(veh_error_data, false, veh_id); //t = sim_time
        veh_list.push_back(vehicle (veh_id, max_speed, veh_type, front_pos, set_speed, veh_error_data.pos_x, veh_error_data.pos_y, veh_error_data.speed, veh_error_data.heading, override, platooning, driver_accel));
        std::cout << "Vehicle Initialized! \n";

        #ifdef VIS_FILE_ENABLE
        write_sim_veh_data(sim_vehs[i], veh_id);
        #endif
    }
    std::cout << "All Vehicles Initialized! \n";
    #ifdef VIS_FILE_ENABLE
    vis_file << ")\n";
    #endif

    while (!sim_done)
    {
        /*
        Note: Everything has been initialized
        Order of Operations:
        1. Publish Data and Send using Callbacks (Int -> Veh, Veh -> Int, Veh -> Veh)
            - this has chance of failing (COMMS_ERROR)
            - should add chance of corruption once CRCs are added
            - during this step, data is written to file for visualization tool
        2. Get Accel Requests from Vehicles
        3. Update Vehicle Data for timestep using existing data + accel requests
            - this should have random error so that system doesnt perfectly fulfil requests
            - additional error to model the sensor errors (GPS, IMU, etc.) is sent to vehicles
                - this is not effecting actual state, just measured state on vehicle callback
        4. Update Vehicles and Intersections for timestep

        Potential Future Additions: 
        - change set speed
            - could be random, or setup in init
        - change driver accel req
            - could be random or setup in init
            - makes it hard to test system
        - change override mode
            - could be random or setup in init
        - change platooning mode
            - could be random or setup in init
        - get vehicle driver alerts (needs to be fleshed out more)
        */

        //1. Publish Data and Send using Callbacks
        //collect int data
        for (int i = 0; i < num_ints; i++)
        {
            int_published[i] = int_list[i].publish();
            print_sim_int_data(int_published[i]); // t = sim_time
        }

        //collect veh data
        for (int i = 0; i < num_vehs; i++)
        {
            veh_published[i] = veh_list[i].publish();
        }

        int comms_pct;
        //send int/veh data to eachother using callbacks
        for (int i = 0; i < num_ints; i++)
        {
            for (int j = 0; j < num_vehs; j++)
            {
                //check if in range for communication
                bool in_range = comms_in_range(int_published[i].int_pos_x, int_published[i].int_pos_y, sim_vehs[j].pos_x, sim_vehs[j].pos_y);
                if (!in_range)
                {
                    //too far, no comms between int and veh
                    continue;
                }

                //send veh data to ints
                comms_pct = (rand() % 100) + 1;
                if (comms_pct > COMMS_ERROR_PCT)
                {
                    //comms work
                    int_list[i].new_vehicle_callback(veh_published[j]);
                }
                else 
                {
                    std::cout << "COMMS ERROR! Receiver: int #" << i + 1 << " \tSender: veh #" << j + 1 << "\n";
                }

                //send int data to vehs
                comms_pct = (rand() % 100) + 1;
                if (comms_pct > COMMS_ERROR_PCT)
                {
                    //comms work
                    veh_list[j].new_int_callback(int_published[i]);
                }
                else 
                {
                    std::cout << "COMMS ERROR! Receiver: veh #" << j + 1 << " \tSender: int #" << i + 1 << "\n";
                }
            }
        }
        //send data between vehicles using callbacks
        for (int i = 0; i < num_vehs; i++)
        {
            for (int j = 0; j < num_vehs; j++)
            {
                if (i == j)
                {
                    //same vehicle, skip
                    continue;
                }

                //check if in range for communication
                bool in_range = comms_in_range(sim_vehs[i].pos_x, sim_vehs[i].pos_y, sim_vehs[j].pos_x, sim_vehs[j].pos_y);
                if (!in_range)
                {
                    //too far, no comms between vehs
                    continue;
                }

                comms_pct = (rand() % 100) + 1;
                if (comms_pct > COMMS_ERROR_PCT)
                {
                    //comms work
                    veh_list[i].new_vehicle_callback(veh_published[j]);
                }
                else 
                {
                    std::cout << "COMMS ERROR! Receiver: veh #" << i + 1 << " \tSender: veh #" << j + 1 << "\n";
                }
            }
        }

        //2. Get Accel Requests from Vehicles
        //3. Update Vehicle Data
        if (sim_time + TIMESTEP <= MAX_TIME)
        {
            std::cout << "\nTime: " << sim_time + TIMESTEP;
            #ifdef VIS_FILE_ENABLE
            vis_file << "(" << sim_time + TIMESTEP;
            #endif
        }
        for (int i = 0; i < num_vehs; i++)
        {
            //get accel req
            sim_vehs[i].accel_req = veh_list[i].get_accel_req();

            //now update vehicle data (with some error)
            sim_vehs[i] = update_veh_data(sim_vehs[i]);

            if (sim_time + TIMESTEP <= MAX_TIME)
            {
                //print vehicle's actual data
                print_sim_veh_data(sim_vehs[i], true, veh_published[i].veh_id); //t = sim_time + TIMESTEP
            }

            //now add error to measured pos, speed, heading (actual values are unchanged)
            s_veh_sim_data veh_error_data = add_veh_error(sim_vehs[i]);

            //print vehicle's measured data
            if (sim_time + TIMESTEP <= MAX_TIME)
            {
                print_sim_veh_data(veh_error_data, false, veh_published[i].veh_id); // t = sim_time + TIMESTEP
            }

            //update dynamics (maybe)
            int ego_data_pct = (rand() % 100) + 1;
            if (ego_data_pct > EGO_DATA_ERROR_PCT)
            {
                //ego data works
                veh_list[i].ego_data_callback(veh_error_data.pos_x, veh_error_data.pos_y, veh_error_data.speed, veh_error_data.heading);
            }
            else 
            {
                std::cout << "EGO DATA ERROR! veh #" << i + 1 << "\n";
            }
        }
        

        //4. Update Vehicles and Intersections
        //update ints
        #ifdef DEBUG
        std::cout << "INTERSECTION DEBUG\n";
        #endif
        for (int i = 0; i < num_ints; i++)
        {
            int_list[i].update(TIMESTEP);
        }

        //update vehs
        #ifdef DEBUG
        std::cout << "VEHICLE DEBUG\n";
        #endif
        for (int i = 0; i < num_vehs; i++)
        {
            veh_list[i].update(TIMESTEP);
        }

        //Write if enabled
        #ifdef VIS_FILE_ENABLE
        if (sim_time + TIMESTEP <= MAX_TIME)
        {
            for (int i = 0; i < num_ints; i++)
            {
                write_sim_int_data(int_list[i].publish());
            }
            for (int i = 0; i < num_vehs; i++)
            {
                write_sim_veh_data(sim_vehs[i], veh_published[i].veh_id);
            }
            vis_file << ")\n";
        }
        #endif

        //Check if sim is finished
        sim_time += TIMESTEP;
        if (sim_time > MAX_TIME)
        {
            sim_done = true;
        }
    }
    std::cout << "Simulation Ended!\n";
    #ifdef VIS_FILE_ENABLE
    vis_file.close();
    #endif
}