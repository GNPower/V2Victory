#include <math.h>
#include <cmath>
#include <iostream>
#include "vehicle.hpp"

#ifdef EMS_OVERRIDE_ENABLE
//need to account for k_veh_type
#ifdef PLATOONING_ENABLE
//Platooning + EMS
vehicle::vehicle(unsigned int id, double max_speed, y_veh_type veh_type, double front_pos, double set_speed, double pos_x, double pos_y, double speed, double heading, bool override, bool platooning, double driver_accel_req)
#else
//EMS
vehicle::vehicle(unsigned int id, double max_speed, y_veh_type veh_type, double front_pos, double set_speed, double pos_x, double pos_y, double speed, double heading, bool override, double driver_accel_req)
#endif
#else
#ifdef PLATOONING_ENABLE
//Platooning
vehicle::vehicle(unsigned int id, double max_speed, double front_pos, double set_speed, double pos_x, double pos_y, double speed, double heading, bool platooning, double driver_accel_req)
#else
//None
vehicle::vehicle(unsigned int id, double max_speed, double front_pos, double set_speed, double pos_x, double pos_y, double speed, double heading, double driver_accel_req)
#endif
#endif
{
    //Setup inputs
    k_id = id;
    k_max_speed = max_speed;
    k_front_pos = front_pos;

    #ifdef EMS_OVERRIDE_ENABLE
    k_veh_type = veh_type;
    #endif
    
    p_set_speed = set_speed;
    validate_set_speed();
    
    #ifdef EMS_OVERRIDE_ENABLE
    if (k_veh_type == EMERGENCY)
    {
        p_override = override;
    }
    else
    {
        p_override = false;
    }
    #endif

    #ifdef PLATOONING_ENABLE
    p_platooning = platooning;
    #endif

    //NOTE: all position updates after initialization are from the GPS & IMU or sim updates methods, this is just for something to start
    m_ego_pos_x = pos_x;
    m_ego_pos_y = pos_y;
    m_ego_speed = speed;
    m_ego_heading = heading;
    ego_updated = true;

    m_driver_accel_req = driver_accel_req;

    //initialize vehicle arrays
    init_veh_data();

    //initialize intersection arrays
    init_int_data();

    //initialize controlled variables
    init_outputs();

    //initialize lead vehicle info
    init_lead_veh();

    //initialize relevant intersection info
    init_rel_int();
}

double vehicle::get_accel_req(void)
{
    return c_accel_req;
}

bool vehicle::get_driver_alerts(void)
{
    return c_driver_alert;
}

#ifdef SIM
s_published_vehicle_data vehicle::publish(void)
#else
void vehicle::publish(void)
#endif
{
    #ifdef SIM
    //just output the data
    s_published_vehicle_data veh_data;
    veh_data.veh_id = k_id;
    veh_data.veh_pos_x = coasted_ego_pos_x;
    veh_data.veh_pos_y = coasted_ego_pos_y;
    veh_data.veh_speed = coasted_ego_speed;
    veh_data.veh_heading = m_ego_heading;

    #ifdef EMS_OVERRIDE_ENABLE
    veh_data.veh_type = k_veh_type;
    veh_data.override = p_override;
    #endif

    #ifdef PLATOONING_ENABLE
    veh_data.platooning = p_platooning;
    veh_data.platoon_set_speed = c_platoon_set_speed;
    #endif

    return veh_data;

    #else
    //ROS setup here
    #endif
}

#ifdef PLATOONING_ENABLE
void vehicle::platooning_callback(bool platooning)
{
    p_platooning = platooning;
}
#endif

#ifdef EMS_OVERRIDE_ENABLE
void vehicle::override_callback(bool override)
{
    if (override && k_veh_type == EMERGENCY)
    {
        p_override = true;
    }
    else
    {
        p_override = false;
    }
}
#endif

void vehicle::ego_data_callback(double pos_x, double pos_y, double speed, double heading)
{
    m_ego_pos_x = pos_x;
    m_ego_pos_y = pos_y;
    m_ego_speed = speed;
    m_ego_heading = heading;
    ego_updated = true;
}

void vehicle::driver_req_callback(double driver_accel_req)
{
    m_driver_accel_req = driver_accel_req;
}

void vehicle::set_speed_callback(double set_speed)
{
    p_set_speed = set_speed;
    validate_set_speed();
}

#ifdef SIM
void vehicle::new_vehicle_callback(s_published_vehicle_data data)
#else
void vehicle::new_vehicle_callback(const std_msgs::String::ConstPtr& msg)
#endif
{
    #ifndef SIM
    //actually parse data, put into s_vehicle_data struct, then do below
    #endif

    if (num_new_vehicles < MAX_VEHICLES)
    {
        new_vehicles[num_new_vehicles].veh_id = data.veh_id;
        new_vehicles[num_new_vehicles].veh_pos_x = data.veh_pos_x;
        new_vehicles[num_new_vehicles].veh_pos_y = data.veh_pos_y;
        new_vehicles[num_new_vehicles].veh_speed = data.veh_speed;
        new_vehicles[num_new_vehicles].veh_heading = data.veh_heading;
        new_vehicles[num_new_vehicles].time_since_update = 0;

        #ifdef PLATOONING_ENABLE
        new_vehicles[num_new_vehicles].platooning = data.platooning;
        new_vehicles[num_new_vehicles].platoon_set_speed = data.platoon_set_speed;
        #endif

        num_new_vehicles++;
    }
}

#ifdef SIM
void vehicle::new_int_callback(s_published_int_data data)
#else
void vehicle::new_int_callback(const std_msgs::String::ConstPtr& msg)
#endif
{
    #ifndef SIM
    //actually parse data, put into s_vehicle_data struct, then do below
    #endif

    if (num_new_ints < MAX_INTS)
    {
        new_ints[num_new_ints].int_id = data.int_id;
        new_ints[num_new_ints].int_length = data.int_length;
        
        new_ints[num_new_ints].int_1_state = data.int_1_state;
        new_ints[num_new_ints].int_1_next_state = data.int_1_next_state;
        new_ints[num_new_ints].int_1_time_to_switch = data.int_1_time_to_switch;
        new_ints[num_new_ints].int_2_state = data.int_2_state;
        new_ints[num_new_ints].int_2_next_state = data.int_2_next_state;
        new_ints[num_new_ints].int_2_time_to_switch = data.int_2_time_to_switch;
        new_ints[num_new_ints].time_since_update = 0;

        #ifdef STOP_SIGN_ENABLE
        new_ints[num_new_ints].int_type = data.int_type;
        new_ints[num_new_ints].int_next_veh_id = data.int_next_veh_id;
        #endif
        
        //copied from intersection, same logic
        double x, y, angle;
        for (int i = 0; i < 4; i++)
        {
            if (i < 2) //Entrance Direction #1
            {   
                angle = data.ent_dir_1*M_PI/180;
            }
            else //Entrance Direction #2
            {
                angle = data.ent_dir_2*M_PI/180;
            }
                
            double x_delta = data.int_length/2*cos(angle); //x is forward, angle is CCW from North
            double y_delta = data.int_length/2*sin(angle); //y is side, angle is CCW from North

            if (i == 0 || i == 2)
            {
                //0 deg
                x = data.int_pos_x + x_delta;
                y = data.int_pos_y + y_delta;
            }
            else
            {
                //180 deg
                x = data.int_pos_x - x_delta;
                y = data.int_pos_y - y_delta;
            }

            new_ints[num_new_ints].int_ent_pos_x[i] = x;
            new_ints[num_new_ints].int_ent_pos_y[i] = y;
        }

        //setup entrance headings
        if (data.ent_dir_1 >= 180)
        {
            new_ints[num_new_ints].int_ent_headings[0] = data.ent_dir_1 - 180;
        }
        else 
        {
            new_ints[num_new_ints].int_ent_headings[0] = data.ent_dir_1 + 180;
        }

        new_ints[num_new_ints].int_ent_headings[1] = data.ent_dir_1;

        if (data.ent_dir_2 >= 180)
        {
            new_ints[num_new_ints].int_ent_headings[2] = data.ent_dir_2 - 180;
        }
        else 
        {
            new_ints[num_new_ints].int_ent_headings[2] = data.ent_dir_2 + 180;
        }

        new_ints[num_new_ints].int_ent_headings[3] = data.ent_dir_2;
    }
    num_new_ints++;
}

void vehicle::update(double dt)
{
    //coast/update ego data
    update_ego_data(dt);

    //first update vehicle and intersection data with new ones
    update_veh_data(dt);
    update_int_data(dt);

    //next find lead vehicle
    update_lead_veh();

    //find relevant intersection
    update_rel_int();

    //now have all the data needed, can update accel req
    update_accel_req();

    #ifdef PLATOONING_ENABLE
    //lastly update platooning info (if enabled)
    update_platooning();
    #endif
}

#ifdef PLATOONING_ENABLE
void vehicle::update_platooning(void)
{
    if (lead_platooning)
    {
        c_platoon_set_speed = lead_platoon_set_speed;
    }
    else
    {
        c_platoon_set_speed = p_set_speed; //we are head of platoong
    }
}
#endif

void vehicle::update_ego_data(double dt)
{
    //operate under assumption callback for ego data is run before
    if (ego_updated)
    {
        ego_updated = false; //reset so if updated again without callback will coast
        //set coasted to same as callback
        coasted_ego_pos_x = m_ego_pos_x;
        coasted_ego_pos_y = m_ego_pos_y;
        coasted_ego_speed = m_ego_speed;
    }
    else
    {
        //cannot assume constant vel, since accelerating
        //work backwards, first update speed
        //dont directly update coasted_ego_speed since can use to get new speed
        double new_speed = coasted_ego_speed + dt*c_accel_req;
        coasted_ego_pos_x = coasted_ego_pos_x + dt*(coasted_ego_speed + new_speed)/2*cos(M_PI/180*m_ego_heading);
        coasted_ego_pos_y = coasted_ego_pos_y + dt*(coasted_ego_speed + new_speed)/2*sin(M_PI/180*m_ego_heading);
        coasted_ego_speed = new_speed;
    }
}

void vehicle::update_accel_req(void)
{
    //first calculate system's accel request
    update_sys_accel_req();

    //check if any driver overrides apply
    if (m_driver_accel_req >= 0)
    {
        //driver wants to accelerate
        c_accel_req = fmax(m_driver_accel_req, system_accel_req);
    }
    else
    {
        c_accel_req = fmin(m_driver_accel_req, system_accel_req);
    }
}

void vehicle::update_sys_accel_req(void)
{
    //takes in the ego vehicle data, lead vehicle, relevant intersection, set speed, platooning mode (if enabled), to determine accel 

    if (lead_exists && approaching_int)
    {
        double int_accel_req = get_int_accel_req();
        double lead_accel_req = get_lead_accel_req();
        system_accel_req = fmin(int_accel_req, lead_accel_req);
    }
    else if (lead_exists)
    {
        //lead vehicle, no intersection
        system_accel_req = get_lead_accel_req();
    }
    else if (approaching_int)
    {
        //intersection, no lead vehicle
        system_accel_req = get_int_accel_req();
    }
    else
    {
        //no lead vehicle or intersection, go to set speed
        system_accel_req = p_set_speed - coasted_ego_speed;
    }
    
    //saturate requests between valid ranges
    if (system_accel_req > MAX_ACCEL)
    {
        system_accel_req = MAX_ACCEL;
    }
    else if (system_accel_req < -MAX_DECEL)
    {
        system_accel_req = -MAX_DECEL;
    }
}

double vehicle::get_lead_accel_req(void)
{
    double lead_accel_req;
    //get desired gap from lead vehicle
    double desired_lead_gap;
    #ifdef PLATOONING_ENABLE
    if (p_platooning && lead_platooning)
    {
        desired_lead_gap = k_front_pos + PLATOONING_GAP;
    }
    else
    {
        desired_lead_gap = k_front_pos + fmax(LEAD_GAP_DIST, LEAD_GAP_TIME*m_ego_speed);
    }
    #else
    desired_lead_gap = k_front_pos + fmax(LEAD_GAP_DIST, LEAD_GAP_TIME*m_ego_speed);
    #endif
    
    double dist_error = lead_rel_pos_x - desired_lead_gap;
    //since using relative stuff need to use rel_vel_x
    double set_speed_error;
    #ifdef PLATOONING_ENABLE
    if (p_platooning && lead_platooning)
    {
        if (lead_platoon_set_speed < p_set_speed)
        {
            c_driver_alert = true;
            std::cout << "platoon set speed below vehicle's set speed!\n using platoon's set speed...\n";
        }
        set_speed_error = lead_platoon_set_speed - coasted_ego_speed;
    }
    else
    {
        set_speed_error = p_set_speed - coasted_ego_speed;
    }
    #else
    set_speed_error = p_set_speed - coasted_ego_speed;
    #endif
    
    //lead_rel_vel_x = lead_vel_x - coasted_ego_speed
    if (set_speed_error < lead_rel_vel_x)
    {
        //focus on reaching set speed
        lead_accel_req = set_speed_error;
    }
    else
    {
        //focus on closing gap
        //vf = 0 since relative vx
        //vi = lead_rel_vel_x
        //delta_s = gap_error
        //using vf^2 = vi^2 + 2*a*delta_s
        if (dist_error == 0)
        {
            lead_accel_req = 0;
        }
        else
        {
            lead_accel_req = pow(lead_rel_vel_x, 2)/(2*dist_error);
        }
    }
    //This probably doesn't work, very unsure how to determine acceleration
    
    return lead_accel_req;
}

double vehicle::get_int_accel_req(void)
{
    double int_accel_req;
    #ifdef STOP_SIGN_ENABLE
    if (int_type == SIGNAL && int_state == RED)
    #else
    if (int_state == RED)
    #endif
    {
        //traffic signal red, check if turning green soon
        if (int_next_state == GREEN)
        {
            //should be green, but need to account for ems override/dynamic timing
            //try to arrive at intersection with min accel
            //using: delta_s = vi*delta_t + a/2*delta_t^2
            //add safety factor to tts to ensure not entering on red light
            double safe_tts = int_time_to_switch + TTS_SAFETY_FACTOR;
            double make_light_req = (int_ent_dist - k_front_pos - coasted_ego_speed*safe_tts)*2/pow(safe_tts, 2);

            //need to ensure we dont go over set speed, so compare vf with set speed 
            double vf = coasted_ego_speed + make_light_req*int_time_to_switch;
            if (vf > p_set_speed || make_light_req > MAX_ACCEL)
            {
                //need to reduce req
                //using: vf^2 = vi^2 + 2*a*delta_s where vf is set_speed
                int_accel_req = (pow(p_set_speed, 2) - pow(coasted_ego_speed, 2))/(2*(int_ent_dist - k_front_pos));
            }
            else
            {
                //request is below set speed at entrance, ok to use
                int_accel_req = make_light_req;
            }
        }
        else
        {
            //next state is red/yellow, just try to approach and stop at intersection
            //using: vf^2 = vi^2 + 2*a*delta_s where vf^2 is 0 and solving for a
            int_accel_req = -pow(coasted_ego_speed, 2)/(2*(int_ent_dist - k_front_pos));
        }
    }
    #ifdef STOP_SIGN_ENABLE
    else if (int_type == SIGNAL)
    #else
    else
    #endif
    {
        //traffic signal green or yellow, check if can make through int or should slow down
        double time_to_red = int_time_to_switch;
        switch(int_next_state)
        {
            case GREEN:
                time_to_red += MIN_RED_GREEN_TIME;
                break;
            case YELLOW:
                time_to_red += MIN_YELLOW;
                break;
            //for red dont increment
        }
        //now have time til red, see if we can exit intersection
        double exit_light_req = (int_exit_dist - k_front_pos - coasted_ego_speed*time_to_red)*2/pow(time_to_red, 2);
        //now check if we go over set speed to reach it
        double vf = coasted_ego_speed + time_to_red*exit_light_req;
        if (vf > p_set_speed || exit_light_req > MAX_ACCEL)
        {
            //can't make it through intersection, need to stop before it
            int_accel_req = -pow(coasted_ego_speed, 2)/(2*(int_ent_dist - k_front_pos));
        }
        else
        {
            //this will barely make it through light, use set speed
            int_accel_req = p_set_speed - coasted_ego_speed;
        }
    }
    #ifdef STOP_SIGN_ENABLE
    else
    {
        //stop sign
        //stop at intersection
        if (int_next_id == k_id)
        {
            //go
            int_accel_req = p_set_speed - coasted_ego_speed;
        }
        else
        {
            //stop at intersection
            int_accel_req = -pow(coasted_ego_speed, 2)/(2*(int_ent_dist - k_front_pos));
        }
    }
    #endif
    return int_accel_req;
}

void vehicle::update_veh_data(double dt)
{
    //first update existing tracks and delte ones that have coasted too long
    for (int i = 0; i < num_vehicles; i++)
    {
        bool match_found = false;
        unsigned int existing_id = m_vehicle_data[i].veh_id;
        for (int j = 0; j < num_new_vehicles; j++)
        {
            if (existing_id == new_vehicles[j].veh_id)
            {
                match_found = true;
                m_vehicle_data[i] = new_vehicles[j];
                break;
            }
        }
        if (!match_found)
        {
            //no update, increase time since last update
            m_vehicle_data[i].time_since_update += dt;

            //check if coasted too long
            if (m_vehicle_data[i].time_since_update > MAX_COASTING)
            {
                //delete, coasted too long
                for (int j = i; j < (num_vehicles - 1); j++)
                {
                    m_vehicle_data[j] = m_vehicle_data[j + 1];
                }
                //reset last vehicle
                m_vehicle_data[num_vehicles - 1].veh_id = 0;
                m_vehicle_data[num_vehicles - 1].veh_pos_x = 0;
                m_vehicle_data[num_vehicles - 1].veh_pos_y = 0;
                m_vehicle_data[num_vehicles - 1].veh_speed = 0;
                m_vehicle_data[num_vehicles - 1].veh_heading = 0;
                m_vehicle_data[num_vehicles - 1].time_since_update = 0;

                #ifdef PLATOONING_ENABLE
                m_vehicle_data[num_vehicles - 1].platooning = false;
                m_vehicle_data[num_vehicles - 1].platoon_set_speed = 0;
                #endif

                num_vehicles--;
                i--; //need to check new m_vehicle_data[i] since shifted over
            }
        }
    }

    //now add new vehicles
    for (int i = 0; i < num_new_vehicles; i++)
    {
        bool match_found = false;
        unsigned int new_id = new_vehicles[i].veh_id;
        for (int j = 0; j < num_vehicles; j++)
        {
            if (new_id == m_vehicle_data[j].veh_id)
            {
                match_found = true;
                break;
            }
        }
        if (!match_found && num_vehicles < MAX_VEHICLES)
        {
            m_vehicle_data[num_vehicles] = new_vehicles[i];
            num_vehicles++;
        }
    }

    //now that new vehicles are added, can reset queue
    reset_new_veh();
}

void vehicle::reset_new_veh(void)
{
    num_new_vehicles = 0;
    for (int i = 0; i < MAX_VEHICLES; i++)
    {
        new_vehicles[i].veh_id = 0;
        new_vehicles[i].veh_pos_x = 0;
        new_vehicles[i].veh_pos_y = 0;
        new_vehicles[i].veh_speed = 0;
        new_vehicles[i].veh_heading = 0;
        new_vehicles[i].time_since_update = 0;
        
        #ifdef PLATOONING_ENABLE
        new_vehicles[i].platooning = false;
        new_vehicles[i].platoon_set_speed = 0;
        #endif
    }
}

void vehicle::update_int_data(double dt)
{
    //first update existing ints, delete ones that have coasted too long
    for (int i = 0; i < num_ints; i++)
    {
        bool match_found = false;
        unsigned int existing_id = m_int_data[i].int_id;
        for (int j = 0; j < num_new_ints; j++)
        {
            if (existing_id == new_ints[j].int_id)
            {
                //update
                match_found = true;
                m_int_data[i] = new_ints[j];
                break;
            }
        }
        if (!match_found)
        {
            //no update, increase time since last update
            m_int_data[i].time_since_update += dt;
            if (m_int_data[i].time_since_update > MAX_COASTING)
            {
                for (int j = i; j < (num_vehicles - 1); j++)
                {
                    m_int_data[j] = m_int_data[j + 1];
                }
                //reset last int
                m_int_data[num_ints - 1].int_id = 0;

                for (int j = 0; j < 4; j++)
                {
                    m_int_data[num_ints - 1].int_ent_pos_x[j] = 0;
                    m_int_data[num_ints - 1].int_ent_pos_y[j] = 0;
                    m_int_data[num_ints - 1].int_ent_headings[j] = 0;
                }
                m_int_data[num_ints - 1].int_length = 0;

                //initialize to GREEN, not to be used since num_ints = 0
                m_int_data[num_ints - 1].int_1_state = GREEN;
                m_int_data[num_ints - 1].int_1_next_state = GREEN;
                m_int_data[num_ints - 1].int_1_time_to_switch = 0;
                m_int_data[num_ints - 1].int_2_state = GREEN;
                m_int_data[num_ints - 1].int_2_next_state = GREEN;
                m_int_data[num_ints - 1].int_2_time_to_switch = 0;

                #ifdef STOP_SIGN_ENABLE
                m_int_data[num_ints - 1].int_type = SIGNAL;
                m_int_data[num_ints - 1].int_next_veh_id = 0;
                #endif

                num_ints--;
                i--;
            }
        }
    }

    //now add new ints
    for (int i = 0; i < num_new_ints; i++)
    {
        bool match_found = false;
        unsigned int new_id = new_ints[i].int_id;
        for (int j = 0; j < num_ints; j++)
        {
            if (new_id == m_int_data[j].int_id)
            {
                match_found = true;
                break;
            }
        }
        if (!match_found && num_ints < MAX_INTS)
        {
            m_int_data[num_ints] = new_ints[i];
            num_ints++;
        }
    }
    //now that new ints are added, can reset queue
    reset_new_int();
}

void vehicle::reset_new_int(void)
{
    num_new_ints = 0;
    for (int i = 0; i < MAX_INTS; i++)
    {
        new_ints[i].int_id = 0;

        for (int j = 0; j < 4; j++)
        {
            new_ints[i].int_ent_pos_x[j] = 0;
            new_ints[i].int_ent_pos_y[j] = 0;
            new_ints[i].int_ent_headings[j] = 0;
        }
        new_ints[i].int_length = 0;

        //initialize to GREEN, not to be used since num_ints = 0
        new_ints[i].int_1_state = GREEN;
        new_ints[i].int_1_next_state = GREEN;
        new_ints[i].int_1_time_to_switch = 0;
        new_ints[i].int_2_state = GREEN;
        new_ints[i].int_2_next_state = GREEN;
        new_ints[i].int_2_time_to_switch = 0;

        #ifdef STOP_SIGN_ENABLE
        new_ints[i].int_type = SIGNAL;
        new_ints[i].int_next_veh_id = 0;
        #endif
    }
}

void vehicle::update_lead_veh(void)
{
    //need to find closest vehicle (if any) in path
    //since we have cars as points but in reality have width, need to account for that when searching

    bool lead_found;
    double lead_dist;
    s_vehicle_data lead_vehicle;

    for (int i = 0; i < num_vehicles; i++)
    {
        //first check if in path
        //coast x and y of track
        double coasted_x = m_vehicle_data[i].veh_pos_x + m_vehicle_data[i].time_since_update*m_vehicle_data[i].veh_speed*cos(M_PI/180*m_vehicle_data[i].veh_heading);
        double coasted_y = m_vehicle_data[i].veh_pos_y + m_vehicle_data[i].time_since_update*m_vehicle_data[i].veh_speed*sin(M_PI/180*m_vehicle_data[i].veh_heading);
        
        double x1, x2, y1, y2, angle_1, angle_2;
        angle_1 = m_ego_heading + 90;
        angle_2 = m_ego_heading - 90;
        x1 = coasted_ego_pos_x + INPATH_WIDTH*cos(M_PI/180*angle_1);
        x2 = coasted_ego_pos_x + INPATH_WIDTH*cos(M_PI/180*angle_2);
        y1 = coasted_ego_pos_y + INPATH_WIDTH*sin(M_PI/180*angle_1);
        y2 = coasted_ego_pos_y + INPATH_WIDTH*sin(M_PI/180*angle_2);

        //to check if in path, solve for x at the coasted_x position based on heading
        double sol_x1, sol_x2;
        sol_x1 = (coasted_x - x1)/cos(M_PI/180*m_ego_heading);
        sol_x2 = (coasted_x - x2)/cos(M_PI/180*m_ego_heading);
        if (sol_x1 <= 0 || sol_x2 <= 0)
        {
            //if these are negative, it is in reverse path (behind us)
            continue;
        }
        //now have x, can see where y is at this x val
        double sol_y1, sol_y2;
        sol_y1 = y1 + sol_x1*sin(M_PI/180*m_ego_heading);
        sol_y2 = y2 + sol_x2*sin(M_PI/180*m_ego_heading);

        //now see if our coasted_y is between the sol_y1 and sol_y2
        if ((coasted_y >= sol_y1 && coasted_y <= sol_y2) || (coasted_y >= sol_y2 && coasted_y <= sol_y1))
        {
            //in path, get distance
            double dist_x = coasted_x - coasted_ego_pos_x;
            double dist_y = coasted_y - coasted_ego_pos_y;
            double dist = pow(dist_x, 2) + pow(dist_y, 2); //dont need to square since using for comparison
            
            if (!lead_found)
            {
                lead_found = true;
                lead_dist = dist;
                lead_vehicle = m_vehicle_data[i];
            }
            else if (lead_found && lead_dist > dist)
            {
                lead_dist = dist;
                lead_vehicle = m_vehicle_data[i];
            }
        }
    }  

    //now have lead, just need to convert to ego's coords (pos x and vel x)
    if (lead_found)
    {
        lead_exists = true;

        //need to get rel pos x and rel vel x
        double x_diff = lead_vehicle.veh_pos_x - coasted_ego_pos_x;
        double y_diff = lead_vehicle.veh_pos_y - coasted_ego_pos_y;
        double diff = pow(x_diff, 2) + pow(y_diff, 2);

        //need to check these calculations
        lead_rel_pos_x = sqrt(diff)*cos(atan(y_diff/x_diff) - M_PI/180*m_ego_heading);

        double heading_diff = lead_vehicle.veh_heading - m_ego_heading;
        lead_rel_vel_x = lead_vehicle.veh_speed*cos(M_PI/180*heading_diff) - coasted_ego_speed; 

        #ifdef PLATOONING_ENABLE
        lead_platooning = lead_vehicle.platooning;
        lead_platoon_set_speed = lead_vehicle.platoon_set_speed;
        #endif
    }
    else
    {
        lead_exists = false;
        lead_rel_pos_x = 0;
        lead_rel_vel_x = 0;

        #ifdef PLATOONING_ENABLE
        lead_platooning = false;
        lead_platoon_set_speed = 0;
        #endif
    }
}

void vehicle::update_rel_int(void)
{
    bool int_found;
    double closest_int_dist;
    double closest_int_length;
    double int_update_time;

    for (int i = 0; i < num_ints; i++)
    {
        //check each entrance
        for (int j = 0; j < 4; j++)
        {
            double heading_diff = abs(m_int_data[i].int_ent_headings[j] - m_ego_heading);
            if (heading_diff > 180)
            {
                heading_diff = 360 - heading_diff;
            }
            if (heading_diff < INLANE_HEADING)
            {
                double x_dist = m_int_data[i].int_ent_pos_x[j] - coasted_ego_pos_x;
                double y_dist = m_int_data[i].int_ent_pos_y[j] - coasted_ego_pos_y;
                double dist = pow(x_dist, 2) + pow(y_dist, 2); //dont need to square since just for comparing
                if (!int_found)
                {
                    int_found = true;
                    closest_int_dist = dist;
                    closest_int_length = m_int_data[i].int_length;
                    int_update_time = m_int_data[i].time_since_update;
                    #ifdef STOP_SIGN_ENABLE
                    int_type = m_int_data[i].int_type;
                    int_next_id = m_int_data[i].int_next_veh_id;
                    #endif

                    if (j < 2)
                    {
                        //Direction #1
                        int_state = m_int_data[i].int_1_state;
                        int_next_state = m_int_data[i].int_1_next_state;
                        int_time_to_switch = m_int_data[i].int_1_time_to_switch;
                    }
                    else
                    {
                        //Direction #2
                        int_state = m_int_data[i].int_2_state;
                        int_next_state = m_int_data[i].int_2_next_state;
                        int_time_to_switch = m_int_data[i].int_2_time_to_switch;
                    }
                    
                }
                else if (int_found && closest_int_dist > dist)
                {
                    closest_int_dist = dist;
                    closest_int_length = m_int_data[i].int_length;
                    int_update_time = m_int_data[i].time_since_update;
                    #ifdef STOP_SIGN_ENABLE
                    int_type = m_int_data[i].int_type;
                    int_next_id = m_int_data[i].int_next_veh_id;
                    #endif

                    if (j < 2)
                    {
                        //Direction #1
                        int_state = m_int_data[i].int_1_state;
                        int_next_state = m_int_data[i].int_1_next_state;
                        int_time_to_switch = m_int_data[i].int_1_time_to_switch;
                    }
                    else
                    {
                        //Direction #2
                        int_state = m_int_data[i].int_2_state;
                        int_next_state = m_int_data[i].int_2_next_state;
                        int_time_to_switch = m_int_data[i].int_2_time_to_switch;
                    }
                }
            }
        }
    }
    approaching_int = int_found;
    if (int_found)
    {
        int_ent_dist = closest_int_dist;
        int_exit_dist = closest_int_dist + closest_int_length;
        int_time_to_switch -= int_update_time;
        #ifdef STOP_SIGN_ENABLE
        if (int_type == SIGNAL && int_time_to_switch < 0) //only care if signaled light
        #else
        if (int_time_to_switch < 0)
        #endif
        {
            //light switched based on last received data, need to account for
            switch (int_next_state)
            {
                //since int_time_to_switch is negative this removes the extra time from dt
                case GREEN:
                    int_state = GREEN;
                    int_next_state = YELLOW;
                    int_time_to_switch = int_time_to_switch + MIN_RED_GREEN_TIME;
                    break;
                case YELLOW:
                    int_state = YELLOW;
                    int_next_state = RED;
                    int_time_to_switch = int_time_to_switch + MIN_YELLOW;
                    break;
                case RED:
                    int_state = RED;
                    int_next_state = GREEN;
                    int_time_to_switch = int_time_to_switch + MIN_RED_GREEN_TIME;
                    break;
            }
        }
    }
    else
    {
        int_ent_dist = 0;
        int_exit_dist = 0;
        int_state = GREEN;
        int_next_state = GREEN;
        int_time_to_switch = 0;

        #ifdef STOP_SIGN_ENABLE
        int_type = SIGNAL;
        int_next_id = 0;
        #endif
    }    
}

void vehicle::init_veh_data(void)
{
    num_vehicles = 0;
    num_new_vehicles = 0;

    for (int i = 0; i < MAX_VEHICLES; i++)
    {
        //existing vehicles
        m_vehicle_data[i].veh_id = 0;
        m_vehicle_data[i].veh_pos_x = 0;
        m_vehicle_data[i].veh_pos_y = 0;
        m_vehicle_data[i].veh_speed = 0;
        m_vehicle_data[i].veh_heading = 0;

        m_vehicle_data[i].time_since_update = 0;

        #ifdef PLATOONING_ENABLE
        m_vehicle_data[i].platooning = false;
        m_vehicle_data[i].platoon_set_speed = 0;
        #endif

        //new vehicles
        new_vehicles[i] = m_vehicle_data[i];
    }
}

void vehicle::init_int_data(void)
{
    num_ints = 0;
    num_new_ints = 0;

    for (int i = 0; i < MAX_INTS; i++)
    {
        //existing intersections
        m_int_data[i].int_id = 0;

        for (int j = 0; j < 4; j++)
        {
            m_int_data[i].int_ent_pos_x[j] = 0;
            m_int_data[i].int_ent_pos_y[j] = 0;
            m_int_data[i].int_ent_headings[j] = 0;
        }
        m_int_data[i].int_length = 0;

        //initialize to GREEN, not to be used since num_ints = 0
        m_int_data[i].int_1_state = GREEN;
        m_int_data[i].int_1_next_state = GREEN;
        m_int_data[i].int_1_time_to_switch = 0;
        m_int_data[i].int_2_state = GREEN;
        m_int_data[i].int_2_next_state = GREEN;
        m_int_data[i].int_2_time_to_switch = 0;

        #ifdef STOP_SIGN_ENABLE
        m_int_data[i].int_type = SIGNAL;
        m_int_data[i].int_next_veh_id = 0;
        #endif

        //new intersections
        new_ints[i] = m_int_data[i];
    }
}

void vehicle::init_outputs(void)
{
    system_accel_req = 0;
    c_accel_req = 0;
    c_driver_alert = false;
    c_platoon_set_speed = 0;
}

void vehicle::init_lead_veh(void)
{
    lead_exists = false;
    lead_rel_pos_x = 0;
    lead_rel_vel_x = 0;

    #ifdef PLATOONING_ENABLE
    lead_platooning = false;
    lead_platoon_set_speed = 0;
    #endif
}

void vehicle::init_rel_int(void)
{
    approaching_int = false;
    int_ent_dist = 0;
    int_exit_dist = 0;
    int_state = GREEN;
    int_next_state = GREEN;
    int_time_to_switch = 0;

    #ifdef STOP_SIGN_ENABLE
    int_type = SIGNAL;
    int_next_id = 0;
    #endif
}

void vehicle::validate_set_speed(void)
{
    if (p_set_speed < MIN_SET_SPEED)
    {
        std::cout << "Set speed below minimum! Setting to minimum...\n";
        p_set_speed = MIN_SET_SPEED;
    }
    else if (p_set_speed > k_max_speed)
    {
        std::cout << "Set speed above maximum vehicle speed! Setting to maximum...\n";
        p_set_speed = k_max_speed;
    }
}