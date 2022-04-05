#include <math.h>
#include <cmath>
#include <iostream>
#include "intersection.hpp"

#ifdef STOP_SIGN_ENABLE
intersection::intersection(double ent_dirs[2], double int_length, double int_pos_x, double int_pos_y, y_int_type int_type, unsigned int id, double ped_countdowns, double time_to_switch[3])
#else
intersection::intersection(double ent_dirs[2], double int_length, double int_pos_x, double int_pos_y, unsigned int id, double ped_countdowns, double time_to_switch[3])
#endif
{
    //Setup constants
    k_ent_dirs[0] = ent_dirs[0];
    k_ent_dirs[1] = ent_dirs[1];
    k_int_length = int_length;
    k_int_pos_x = int_pos_x;
    k_int_pos_y = int_pos_y;
    k_id = id;
    #ifdef STOP_SIGN_ENABLE
    k_int_type = int_type;
    #endif

    //Initialize Intersection Entrance Positions
    init_int_ent_data();

    //States need validation
    p_ped_countdowns = ped_countdowns;
    p_time_to_switch_1[0] = time_to_switch[0];
    p_time_to_switch_1[1] = time_to_switch[1];
    p_time_to_switch_1[2] = time_to_switch[2];

    //Init Intersection
    init_states(); //needed in stopsign for publish
    init_traffic_signal();
    #ifdef STOP_SIGN_ENABLE
    if (k_int_type == STOP)
    {
        init_stop_sign();
    }
    #endif

    //Init Emergency Override Queue
    #ifdef EMS_OVERRIDE_ENABLE
    init_ems_override();
    #endif

    //Init Vehicle Localization
    #if defined(SMART_TIMING_ENABLE) || defined(STOP_SIGN_ENABLE) || defined(EMS_OVERRIDE_ENABLE)
    init_traffic_tracker();
    #endif
}

#ifdef SIM
s_published_int_data intersection::publish(void)
#else
void intersection::publish(void)
#endif
{
    #ifdef SIM
    //just output the data
    s_published_int_data int_data;
    int_data.ent_dir_1 = k_ent_dirs[0];
    int_data.ent_dir_2 = k_ent_dirs[1];
    int_data.int_length = k_int_length;
    int_data.int_pos_x = k_int_pos_x;
    int_data.int_pos_y = k_int_pos_y;
    int_data.int_id = k_id;

    #ifdef STOP_SIGN_ENABLE
    int_data.int_type = k_int_type;
    #endif

    int_data.int_1_state = c_int_1_state;
    int_data.int_1_next_state = c_int_1_next_state;
    int_data.int_1_time_to_switch = c_int_1_time_to_switch;

    int_data.int_2_state = c_int_2_state;
    int_data.int_2_next_state = c_int_2_next_state;
    int_data.int_2_time_to_switch = c_int_2_time_to_switch;

    #ifdef STOP_SIGN_ENABLE
    int_data.int_next_veh_id = c_int_next_veh_id;
    #endif
    
    return int_data;

    #else
    //ROS setup here
    #endif
}

#ifdef SIM
void intersection::new_vehicle_callback(s_published_vehicle_data data)
#else
void intersection::new_vehicle_callback(const std_msgs::String::ConstPtr& msg)
#endif
{
    #ifndef SIM
    //actually parse data, put into new_vehicle_data struct, then do below
    #endif

    if (num_new_vehicles < MAX_VEHICLES)
    {
        new_vehicles[num_new_vehicles].in_int = false;
        new_vehicles[num_new_vehicles].int_entrance = -1;

        new_vehicles[num_new_vehicles].time_since_update = 0;
        new_vehicles[num_new_vehicles].veh_id = data.veh_id;
        new_vehicles[num_new_vehicles].veh_pos_x = data.veh_pos_x;
        new_vehicles[num_new_vehicles].veh_pos_y = data.veh_pos_y;
        new_vehicles[num_new_vehicles].veh_speed = data.veh_speed;
        new_vehicles[num_new_vehicles].veh_heading = data.veh_heading;

        #ifdef EMS_OVERRIDE_ENABLE
        new_vehicles[num_new_vehicles].veh_type = data.veh_type;
        new_vehicles[num_new_vehicles].override = data.override;
        #endif

        num_new_vehicles++;
    }
}

void intersection::update(double dt)
{
    //dt is delta in time since last update
    #if defined(SMART_TIMING_ENABLE) || defined(STOP_SIGN_ENABLE) || defined(EMS_OVERRIDE_ENABLE)
    update_traffic_tracker(dt);
    #endif

    #ifdef EMS_OVERRIDE_ENABLE
    update_ems_override(dt);
    #endif

    #ifdef STOP_SIGN_ENABLE
    if (k_int_type == STOP)
    {
        update_stop_sign(dt);
    }
    else
    {
        update_traffic_signal(dt);
    }
    #else
    update_traffic_signal(dt);
    #endif
}

void intersection::update_traffic_signal(double dt)
{
    //first use dt to update states
    //overall this section requires alot of ifdefs based on the active stretch goals
    //first update FSM, then see if next state, time to switch, or pedestrian countdown should be changed

    //Update FSM
    double dt_remaining;
    //Direction #1
    if (dt >= c_int_1_time_to_switch)
    {
        #ifdef DEBUG
        std::cout << "update_traffic_signal: direction #1 switch state\n";
        #endif

        //switch to next state
        c_int_1_state = c_int_1_next_state;

        dt_remaining = dt - c_int_1_time_to_switch;
        //since next state might not be in order, when switching find info based on switched to state
        for (int i = 0; i < 3; i++)
        {
            if (c_int_1_state == p_states[i])
            {
                //match found, should be unique so no issue
                int next_index = (i + 1) % 3;
                c_int_1_next_state = p_states[next_index];
                c_int_1_time_to_switch = p_time_to_switch_1[i] - dt_remaining;
                if (c_int_1_state == GREEN)
                {
                    c_int_1_ped_countdown = fmin(p_ped_countdowns, c_int_1_time_to_switch);
                }
                else {
                    c_int_1_ped_countdown = 0; //red or yellow light
                }
                break;
            }
        }
    }
    else
    {
        //same state
        c_int_1_time_to_switch -= dt;
        if (c_int_1_state == GREEN)
        {
            c_int_1_ped_countdown = fmin(c_int_1_ped_countdown, c_int_1_time_to_switch);
        }
        else {
            c_int_1_ped_countdown = 0; //red or yellow light
        }
    }

    //Direction #2
    if (dt >= c_int_2_time_to_switch)
    {

        #ifdef DEBUG
        std::cout << "update_traffic_signal: direction #2 switch state\n";
        #endif

        //switch to next state
        c_int_2_state = c_int_2_next_state;

        dt_remaining = dt - c_int_2_time_to_switch;
        //since next state might not be in order, when switching find based on switched to state
        for (int i = 0; i < 3; i++)
        {
            if (c_int_2_state == p_states[i])
            {
                //match found, should be unique so no issue
                int next_index = (i + 1) % 3;
                c_int_2_next_state = p_states[next_index];
                c_int_2_time_to_switch = p_time_to_switch_2[i] - dt_remaining;
                if (c_int_2_state == GREEN)
                {
                    c_int_2_ped_countdown = fmin(p_ped_countdowns, c_int_2_time_to_switch);
                }
                else {
                    c_int_2_ped_countdown = 0; //red or yellow light
                }
                break;
            }
        }
    }
    else
    {
        //same state
        c_int_2_time_to_switch -= dt;
        if (c_int_2_state == GREEN)
        {
            c_int_2_ped_countdown = fmin(c_int_2_ped_countdown, c_int_2_time_to_switch);
        }
        else {
            c_int_2_ped_countdown = 0; //red or yellow light
        }
    }

    //now check if EMS enabled/active
    #ifdef EMS_OVERRIDE_ENABLE
    if (ems_override_active)
    {
       if (ems_override_entrance == 0 || ems_override_entrance == 1)
       {
            #ifdef DEBUG
            std::cout << "update_traffic_signal: direction #1 EMS override active\n";
            #endif
            //Direction #1
            if (c_int_1_state == GREEN && c_int_2_state == RED)
            {
                //Direction #1 is green, Direction #2 should be red
                //keep same states
                c_int_1_next_state = GREEN;
                c_int_2_next_state = RED;
            }
            else if (c_int_1_state == RED)
            {
                //Direction #1 is red, Direction #2 could be yellow or green
                //switch states if dir #2 is green, if yellow will be red next so ok
                if (c_int_2_state == GREEN)
                {
                    //need to reduce times equally
                    c_int_1_time_to_switch -= c_int_2_time_to_switch;

                    //set from GREEN to YELLOW
                    c_int_2_state = YELLOW;
                    for (int i = 0; i < 3; i++)
                    {
                        if (c_int_2_state == p_states[i])
                        {
                            //match found, should be unique so no issue
                            int next_index = (i + 1) % 3;
                            c_int_2_next_state = p_states[next_index];
                            c_int_2_time_to_switch = p_time_to_switch_2[i];
                            c_int_2_ped_countdown = 0;
                            break;
                        }
                    }
                }
            }
            else
            {
                //Direction #1 is yellow, Direction #2 should be red
                c_int_1_next_state = GREEN;
                c_int_2_next_state = RED;
            }
       } 
       else
       {
            #ifdef DEBUG
            std::cout << "update_traffic_signal: direction #2 EMS override active\n";
            #endif
           //Direction #2
           if (c_int_2_state == GREEN && c_int_1_state == RED)
           {
                //Direction #2 is green, Direction #1 should be red
                //keep same states
                c_int_1_next_state = RED;
                c_int_2_next_state = GREEN;
           }
           else if (c_int_2_state == RED)
           {
               //Direction #2 is red, Direction #1 could be yellow or green
               //switch states/times if dir #1 is green, if yellow will be red next so ok
               if (c_int_1_state == GREEN)
               {
                   //need to reduce times equally
                    c_int_2_time_to_switch -= c_int_1_time_to_switch;

                    //set from GREEN to YELLOW
                    c_int_1_state = YELLOW;
                    for (int i = 0; i < 3; i++)
                    {
                        if (c_int_1_state == p_states[i])
                        {
                            //match found, should be unique so no issue
                            int next_index = (i + 1) % 3;
                            c_int_1_next_state = p_states[next_index];
                            c_int_1_time_to_switch = p_time_to_switch_1[i];
                            c_int_1_ped_countdown = 0;
                            break;
                        }
                    }
               }
           }
           else
            {
                //Direction #2 is yellow, Direction #1 should be red
                c_int_2_next_state = GREEN;
                c_int_1_next_state = RED;
            }
       }
    }
    #endif

    //lastly check if dynamic timing enabled/active
    //similar to EMS except limited by ped countdown value
    #ifdef SMART_TIMING_ENABLE
    #ifdef EMS_OVERRIDE_ENABLE
    if (priority_enable && !ems_override_active)
    #else
    if (priority_enable)
    #endif
    {
        //check if direction to give priority is red or not and direction to lose priority is green
        //also can only set to ped countdown, so need to check if possible to reduce
        if (priority_direction == 0 && c_int_1_state == RED && c_int_2_state == GREEN && c_int_2_time_to_switch > c_int_2_ped_countdown)
        {
            #ifdef DEBUG
            std::cout << "update_traffic_signal: direction #1 prioritized\n";
            #endif

            //Direction #1
            //need to account for difference
            double time_diff = c_int_2_time_to_switch - c_int_2_ped_countdown;
            c_int_2_time_to_switch = c_int_2_ped_countdown;
            c_int_1_time_to_switch -= time_diff;
        }
        else if (priority_direction == 1 && c_int_2_state == RED && c_int_1_state == GREEN && c_int_1_time_to_switch > c_int_1_ped_countdown)
        {
            #ifdef DEBUG
            std::cout << "update_traffic_signal: direction #2 prioritized\n";
            #endif

            //Direction #2
            //need to account for difference
            double time_diff = c_int_1_time_to_switch - c_int_1_ped_countdown;
            c_int_1_time_to_switch = c_int_1_ped_countdown;
            c_int_2_time_to_switch -= time_diff;
        }
    }
    #endif
}

#ifdef STOP_SIGN_ENABLE
void intersection::update_stop_sign(double dt)
{
    //this logic is very simple since most of it is covered else where
    #ifdef EMS_OVERRIDE_ENABLE
    if (ems_override_active)
    {
        c_int_next_veh_id = ems_override_active_id;
    }
    else
    {
        c_int_next_veh_id = next_stopped_veh_id;
    }
    #else
    c_int_next_veh_id = next_stopped_veh_id;
    #endif
}
#endif

#ifdef EMS_OVERRIDE_ENABLE
void intersection::update_ems_override(double dt)
{
    //check and update queue + new overrides + current one
    
    //update queue of overrides + check current active one (if one is active)
    bool match_found;
    bool active_ems_found = false;
    for (int i = 0; i < num_overrides; i++)
    {
        unsigned int override_id = ems_override_queue[i];

        match_found = false;
        for (int j = 0; j < num_vehicles; j++)
        {
            if (override_id == m_vehicle_data[j].veh_id)
            {
                match_found = true;

                if(ems_override_active && override_id == ems_override_active_id)
                {
                    active_ems_found = true;
                }

                if (!m_vehicle_data[j].override || m_vehicle_data[j].int_entrance == -1 || m_vehicle_data[j].time_since_update > EMS_COASTING)
                {
                    //if override is deactivated or exited intersection, or not updated then remove from queue
                    
                    for (int k = i; k < (num_overrides - 1); k++)
                    {
                        ems_override_queue[k] = ems_override_queue[k + 1];
                    }
                    //lastly reset final override
                    ems_override_queue[num_overrides - 1] = 0;

                    num_overrides--;
                    i--;
                    

                    if (ems_override_active && override_id == ems_override_active_id)
                    {
                        //active override, matching ID, need to deactivate
                        ems_override_active = false;
                        ems_override_active_id = 0;
                    }
                }
                break;
            }
        }
    }
    if (ems_override_active && !active_ems_found)
    {
        ems_override_active = false;
        ems_override_active_id = 0;
    }

    //now add new overrides
    s_override_vehicle new_overrides[MAX_VEHICLES];
    int num_new_overrides = 0;
    for (int i = 0; i < num_vehicles; i++)
    {
        if (m_vehicle_data[i].override && m_vehicle_data[i].int_entrance != -1 && m_vehicle_data[i].time_since_update <= EMS_COASTING && !m_vehicle_data[i].in_int)
        {
            //should not be in intersection if new
            //if new, time_since_update should be 0
            bool match_found = false;
            for (int j = 0; j < num_overrides; j++)
            {
                if (m_vehicle_data[i].veh_id == ems_override_queue[j])
                {
                    match_found = true;
                    break;
                }
            }
            if (!match_found && num_new_overrides < MAX_VEHICLES) //new override
            {
                new_overrides[num_new_overrides].veh_id = m_vehicle_data[i].veh_id;
                double dist_x = m_vehicle_data[i].veh_pos_x - int_ent_pos_x[m_vehicle_data[i].int_entrance];
                double dist_y = m_vehicle_data[i].veh_pos_y - int_ent_pos_y[m_vehicle_data[i].int_entrance];
                new_overrides[num_new_overrides].int_dist = pow(dist_x, 2) + pow(dist_y, 2);
                num_new_overrides++;
            }
        }
    }
    //now for new overrides sort and add to existing
    if (num_new_overrides > 1)
    {
        //multiple, need to sort
        for (int i = 0; i < num_new_overrides; i++)
        {
            for(int j = 0; j < (num_new_overrides - i - 1); j++)
            {
                if (new_overrides[j].int_dist > new_overrides[j + 1].int_dist)
                {
                    s_override_vehicle temp_override = new_overrides[j];
                    new_overrides[j] = new_overrides[j + 1];
                    new_overrides[j + 1] = temp_override;
                }
            }
        }
        //now sorted
        for (int i = 0; i < num_new_overrides; i++)
        {
            ems_override_queue[num_overrides] = new_overrides[i].veh_id;
            num_overrides++;
        }
    }
    else if (num_new_overrides == 1) //don't need to sort
    {
        ems_override_queue[num_overrides] = new_overrides[0].veh_id;
        num_overrides++;
    }

    //lastly, if no override active, see if any in queue
    if (!ems_override_active && num_overrides > 0)
    {
        ems_override_active = true;
        ems_override_active_id = ems_override_queue[0];
        for (int i = 0; i < num_vehicles; i++)
        {
            if (ems_override_active_id == m_vehicle_data[i].veh_id)
            {
                ems_override_entrance = m_vehicle_data[i].int_entrance;
                break;
            }
        }
    }
}
#endif

#if defined(SMART_TIMING_ENABLE) || defined(STOP_SIGN_ENABLE) || defined(EMS_OVERRIDE_ENABLE)
void intersection::update_traffic_tracker(double dt)
{
    //first update existing tracks, delete ones that have been coasted too long
    for (int i = 0; i < num_vehicles; i++)
    {
        bool match_found = false;
        unsigned int existing_id = m_vehicle_data[i].veh_id;
        for (int j = 0; j < num_new_vehicles; j++)
        {
            if (existing_id == new_vehicles[j].veh_id)
            {
                //update existing
                match_found = true;
                m_vehicle_data[i] = new_vehicles[j];
                break;
            }
        }
        if (!match_found)
        {
            //existing track not updated, increase time since last update
            m_vehicle_data[i].time_since_update += dt;
            if (m_vehicle_data[i].time_since_update > MAX_COASTING)
            {
                for (int j = i; j < (num_vehicles - 1); j++)
                {
                    m_vehicle_data[j] = m_vehicle_data[j + 1];
                }
                //lastly reset final vehicle
                m_vehicle_data[num_vehicles - 1].veh_id = 0;
                m_vehicle_data[num_vehicles - 1].veh_pos_x  = 0;
                m_vehicle_data[num_vehicles - 1].veh_pos_y = 0;
                m_vehicle_data[num_vehicles - 1].veh_speed = 0;
                m_vehicle_data[num_vehicles - 1].veh_heading = 0;
                m_vehicle_data[num_vehicles - 1].time_since_update = 0;
                m_vehicle_data[num_vehicles - 1].in_int = false;
                m_vehicle_data[num_vehicles - 1].int_entrance = -1;

                #ifdef EMS_OVERRIDE_ENABLE
                m_vehicle_data[num_vehicles - 1].veh_type = NORMAL;
                m_vehicle_data[num_vehicles - 1].override = false;
                #endif

                num_vehicles--;
                i--; //i-- since we haven't checked for update of this new vehicle data that replaced the old one
            }
        }
    }

    //lastly add new tracks
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
            //new track, add to array
            m_vehicle_data[num_vehicles] = new_vehicles[i];
            num_vehicles++;
        }
    }

    //new tracks are added, now reset new_vehicles for future callbacks
    reset_new_vehicles();

    //now assign intersection entrance (-1 means already exited intersection)
    assign_int_ents();

    #ifdef STOP_SIGN_ENABLE
    if (k_int_type == SIGNAL)
    {
        #ifdef SMART_TIMING_ENABLE
        update_traffic_priority();
        #endif
    }
    else //stop sign
    {
        update_stop_sign_priority(dt);
    }
    #else //no stop signs enabled
    #ifdef SMART_TIMING_ENABLE
    update_traffic_priority();
    #endif
    #endif
}

void intersection::assign_int_ents(void)
{
    for (int i = 0; i < num_vehicles; i++)
    {
        //update position
        double coasted_x, coasted_y;
        coasted_x = m_vehicle_data[i].veh_pos_x + m_vehicle_data[i].time_since_update*m_vehicle_data[i].veh_speed*cos(M_PI/180*m_vehicle_data[i].veh_heading);
        coasted_y = m_vehicle_data[i].veh_pos_y + m_vehicle_data[i].time_since_update*m_vehicle_data[i].veh_speed*sin(M_PI/180*m_vehicle_data[i].veh_heading);

        //determine if vehicle is approaching or exiting intersection
        //if approaching determine from which entrance
        int closest_ent = 0;
        bool approach_ent = false;
        
        //Distance to entrance
        double ent_dist_x = int_ent_pos_x[0] - coasted_x;
        double ent_dist_y = int_ent_pos_y[0] - coasted_y;
        double ent_dist = pow(ent_dist_x,2) + pow(ent_dist_y,2); //don't need sqrt to determine closest
        double closest_ent_dist = ent_dist;

        //need to see if approaching or leaving
        //assume const vel and see if time to reach is positive
        double x_time, y_time;
        if (ent_dist_x == 0)
        {
            x_time = 0;
        }
        else
        {
            x_time = ent_dist_x/cos(M_PI/180*m_vehicle_data[i].veh_heading);
        }
        
        if (ent_dist_y == 0)
        {
            y_time = 0;
        }
        else
        {
            y_time = ent_dist_y/sin(M_PI/180*m_vehicle_data[i].veh_heading);
        }
            
        approach_ent = x_time >= 0 && y_time >= 0;

        //Difference in heading compared to entrance
        double ent_heading = abs(int_ent_headings[0] - m_vehicle_data[i].veh_heading);
        if (ent_heading > 180)
        {
            ent_heading = 360 - ent_heading;
        }
        double closest_ent_heading = ent_heading;

        for (int j = 1; j < 4; j++) //start at 1 since 0 index used in init
        {
            //Calculate Distance
            ent_dist_x = int_ent_pos_x[j] - coasted_x;
            ent_dist_y = int_ent_pos_y[j] - coasted_y;
            ent_dist = pow(ent_dist_x, 2) + pow(ent_dist_y, 2);

            //Calculate Heading
            ent_heading = abs(int_ent_headings[j] - m_vehicle_data[i].veh_heading);
            if (ent_heading > 180)
            {
                ent_heading = 360 - ent_heading;
            }

            if ((closest_ent_dist > ent_dist) || (closest_ent_dist == ent_dist && closest_ent_heading > ent_heading))
            {
                //new closest entrance
                closest_ent = j;
                closest_ent_dist = ent_dist;
                closest_ent_heading = ent_heading;

                //need to see if approaching or leaving
                //assume const vel and see if time to reach is positive
                double x_time, y_time;
                if (ent_dist_x == 0)
                {
                    x_time = 0;
                }
                else
                {
                    x_time = ent_dist_x/cos(M_PI/180*m_vehicle_data[i].veh_heading);
                }
                
                if (ent_dist_y == 0)
                {
                    y_time = 0;
                }
                else
                {
                    y_time = ent_dist_y/sin(M_PI/180*m_vehicle_data[i].veh_heading);
                }
                    
                approach_ent = x_time >= 0 && y_time >= 0;
            }
        }
        //now have closest int, distance to int, heading difference

        //check in intersection:
        //actual check is here
        if (coasted_x > min_x && coasted_x < max_x && coasted_y > min_y && coasted_y < max_y)
        {
            m_vehicle_data[i].in_int = true;
        }
        else
        {
            m_vehicle_data[i].in_int = false;
        }

        if (m_vehicle_data[i].in_int)
        {
            m_vehicle_data[i].int_entrance = closest_ent; 
        }
        else if (closest_ent_heading < INLANE_HEADING) //close enough in heading to closest direction
        {
            m_vehicle_data[i].int_entrance = closest_ent;
        }
        else
        {
            m_vehicle_data[i].int_entrance = -1; //exiting intersection
        }

        #ifdef DEBUG
        std::cout << "assign_int_ents: Vehicle ID: " << m_vehicle_data[i].veh_id << "\tEntrance: " << m_vehicle_data[i].int_entrance << "\tIn Int: " << m_vehicle_data[i].in_int << "\n";
        std::cout << "assign_int_ents: Position X: " << coasted_x << "\t Position Y: " << coasted_y << "\n";
        #endif
    }
}

void intersection::reset_new_vehicles(void)
{
    num_new_vehicles = 0;
    for (int i = 0; i < MAX_VEHICLES; i++)
    {
        new_vehicles[i].veh_id = 0;
        new_vehicles[i].veh_pos_x  = 0;
        new_vehicles[i].veh_pos_y = 0;
        new_vehicles[i].veh_speed = 0;
        new_vehicles[i].veh_heading = 0;
        new_vehicles[i].time_since_update = 0;
        new_vehicles[i].in_int = false;
        new_vehicles[i].int_entrance = -1;

        #ifdef EMS_OVERRIDE_ENABLE
        new_vehicles[i].veh_type = NORMAL;
        new_vehicles[i].override = false;
        #endif
    }
}
#endif

#ifdef SMART_TIMING_ENABLE
void intersection::update_traffic_priority(void)
{
    //this simply checks which direction should be prioritized if any, doesn't mean it will be used

    //first need to assign vehicles to directions
    //remember need to apply coasting for old vehicles
    
    int dir_counts[2] = {0, 0};
    //relating to the int_ent_pos_x and int_ent_pos_y, dir_counts[0] is 0, 1 and dir_counts[1] is 2, 3 (dir_counts matches with k_ent_dirs)

    for (int i = 0; i < num_vehicles; i++)
    {
        if (m_vehicle_data[i].int_entrance == 0 || m_vehicle_data[i].int_entrance == 1)
        {
            dir_counts[0]++;
        }
        else if (m_vehicle_data[i].int_entrance == 2 || m_vehicle_data[i].int_entrance == 3) //can't use else bc -1 represents exiting
        {
            dir_counts[1]++;
        }
    }

    #ifdef DEBUG
    std::cout << "update_traffic_priority: dir_count[0] = " << dir_counts[0] << "\tdir_count[1] = " << dir_counts[1] << "\n";
    #endif

    //have dir_counts, can see if switching is possible
    
    if (dir_counts[0] == 0 && dir_counts[1] == 0) //no cars entering, keep same
    {
        priority_enable = false;
    }
    else if (dir_counts[0] == 0) //no cars entering in dir 1, some in dir 2, should switch
    {
        priority_direction = 1;
        priority_enable = true;
    } 
    else if (dir_counts[1] == 0) //no cars entering in dir 2, some in dir 1, should switch
    {
        priority_direction = 0;
        priority_enable = true;
    }
    else
    {       
        int dir_diff_0 = dir_counts[0] - dir_counts[1];
        double dir_ratio_0 = (double)dir_counts[0]/(double)dir_counts[1];
        int dir_diff_1 = dir_counts[1] - dir_counts[0];
        double dir_ratio_1 = (double)dir_counts[1]/(double)dir_counts[0];


        if (dir_diff_0 >= TRAFFIC_ABS_DIFF || dir_ratio_0 >= TRAFFIC_REL_DIFF) //way more cars in dir 1, should switch
        {
            priority_direction = 0;
            priority_enable = true;
        }
        else if (dir_diff_1 >= TRAFFIC_ABS_DIFF || dir_ratio_1 >= TRAFFIC_REL_DIFF) //way more cars in dir 2, should switch
        {
            priority_direction = 1;
            priority_enable = true;
        }
        else //not a big enough difference in # of cars, keep same
        {
            priority_enable = false;
        }
    }
}
#endif

#ifdef STOP_SIGN_ENABLE
void intersection::update_stop_sign_priority(double dt)
{
    //need to check current queue and see where top vehicle is
    //if too old or exited intersection, then can remove and update next_stopped_veh_id

    //first update existing stopped vehicles, remove ones that are moving
    for (int i = 0; i < num_stopped_vehicles; i++)
    {
        unsigned int existing_id = stopped_vehicles[i].veh_id;
        bool match_found = false;
        for (int j = 0; j < num_vehicles; j++)
        {
            if (existing_id == m_vehicle_data[j].veh_id)
            {
                if (m_vehicle_data[j].veh_speed <= STOPPED_SPEED && m_vehicle_data[j].time_since_update <= MAX_STOP_SIGN_COAST && m_vehicle_data[j].int_entrance != -1 && !m_vehicle_data[j].in_int)
                {
                    //still stopped, recently updated, approaching intersection, not inside intersection
                    stopped_vehicles[i].time_stopped += dt;
                    #ifdef DEBUG
                    std::cout << "update_stop_sign_priority: still stopped id = " << m_vehicle_data[j].veh_id << "\ttime_stopped = " << stopped_vehicles[i].time_stopped << "\n";
                    #endif
                }
                else
                {
                    #ifdef DEBUG
                    std::cout << "update_stop_sign_priority: now moving id = " << m_vehicle_data[j].veh_id << "\n";
                    #endif
                    //started moving or not updated in long time, delete
                    for (int k = i; k < (num_stopped_vehicles - 1); k++)
                    {
                        stopped_vehicles[k] = stopped_vehicles[k + 1];
                    }
                    //lastly reset final vehicle
                    stopped_vehicles[num_stopped_vehicles - 1].veh_id = 0;
                    stopped_vehicles[num_stopped_vehicles - 1].time_stopped = 0;

                    num_stopped_vehicles--;
                    i--;
                }
                match_found = true;
                break;
            }
        }
        if (!match_found)
        {
            #ifdef DEBUG
            std::cout << "update_stop_sign_priority: not found id = " << stopped_vehicles[i].veh_id << "\n";
            #endif
            //delete
            for (int k = i; k < (num_stopped_vehicles - 1); k++)
            {
                stopped_vehicles[k] = stopped_vehicles[k + 1];
            }
            //lastly reset final vehicle
            stopped_vehicles[num_stopped_vehicles - 1].veh_id = 0;
            stopped_vehicles[num_stopped_vehicles - 1].time_stopped = 0;

            num_stopped_vehicles--;
            i--;
        }
    }
    //now add any new vehicles
    for (int i = 0; i < num_vehicles; i++)
    {
        if (m_vehicle_data[i].veh_speed <= STOPPED_SPEED && m_vehicle_data[i].time_since_update <= MAX_STOP_SIGN_COAST && m_vehicle_data[i].int_entrance != -1 && !m_vehicle_data[i].in_int)
        {
            //must be stopped, updated recent enough, not past the intersection and not in the intersection
            bool match_found = false;
            unsigned int new_id = m_vehicle_data[i].veh_id;
            for (int j = 0; j < num_stopped_vehicles; j++)
            {
                if (new_id == stopped_vehicles[j].veh_id)
                {
                    match_found = true;
                    break;
                }
            }
            if (!match_found && num_stopped_vehicles < MAX_VEHICLES)
            {
                #ifdef DEBUG
                std::cout << "update_stop_sign_priority: new stopped id = " << new_id << "\n";
                #endif
                stopped_vehicles[num_stopped_vehicles].veh_id = new_id;
                stopped_vehicles[num_stopped_vehicles].time_stopped = 0;
                num_stopped_vehicles++;
            }
        }
    }

    //check if current next car is still in intersection/stopped or has exited
    bool cur_next_found = false;
    for (int i = 0; i < num_vehicles; i++)
    {
        if (m_vehicle_data[i].veh_id == next_stopped_veh_id)
        {
            cur_next_found = true;
            //now see if its still in the intersection or stopped
            if (m_vehicle_data[i].int_entrance == -1 || m_vehicle_data[i].time_since_update > MAX_STOP_SIGN_COAST)
            {
                next_stopped_veh_id = 0; //can reset, -1 means exiting
            }
            break;
        }
    }

    if (!cur_next_found)
    {
        //not found in vehicles list, assume exited
        next_stopped_veh_id = 0;
    }

    //now update next vehicle if id is currently 0
    if (next_stopped_veh_id == 0 && num_stopped_vehicles > 0)
    {
        #ifdef DEBUG
        std::cout << "update_stop_sign_priority: finding new next veh\n";
        #endif
        double max_time_stopped = STOPPED_TIME;
        for (int i = 0; i < num_stopped_vehicles; i++)
        {
            if (stopped_vehicles[i].time_stopped > max_time_stopped)
            {
                #ifdef DEBUG
                std::cout << "update_stop_sign_priority: new best time stopped = " << stopped_vehicles[i].time_stopped << "\n";
                #endif
                //dont bother checking position if it wont beat current max stopping time
                unsigned int stopped_id = stopped_vehicles[i].veh_id;
                for (int j = 0; j < num_vehicles; j++)
                {
                    if (stopped_id == m_vehicle_data[j].veh_id)
                    {
                        double stop_dist_x = m_vehicle_data[j].veh_pos_x - int_ent_pos_x[m_vehicle_data[j].int_entrance];
                        double stop_dist_y = m_vehicle_data[j].veh_pos_y - int_ent_pos_y[m_vehicle_data[j].int_entrance];
                        double stop_dist = sqrt(pow(stop_dist_x, 2) + pow(stop_dist_y, 2));

                        #ifdef DEBUG
                        std::cout << "update_stop_sign_priority: stop_dist = " << stop_dist << "\n";
                        #endif

                        if (stop_dist <= STOP_SIGN_DISTANCE)
                        {
                            //stopped in range
                            max_time_stopped = stopped_vehicles[i].time_stopped;
                            next_stopped_veh_id = stopped_id;
                        }
                    }
                }
            }
        }
    }
}
#endif

#if defined(SMART_TIMING_ENABLE) || defined(STOP_SIGN_ENABLE) || defined(EMS_OVERRIDE_ENABLE)
void intersection::init_traffic_tracker(void)
{
    #ifdef SMART_TIMING_ENABLE
    priority_enable = false;
    priority_direction = 0;
    #endif

    num_vehicles = 0;
    num_new_vehicles = 0;

    for (int i = 0; i < MAX_VEHICLES; i++)
    {
        //existing vehicles
        m_vehicle_data[i].veh_id = 0;
        m_vehicle_data[i].veh_pos_x  = 0;
        m_vehicle_data[i].veh_pos_y = 0;
        m_vehicle_data[i].veh_speed = 0;
        m_vehicle_data[i].veh_heading = 0;
        m_vehicle_data[i].time_since_update = 0;
        m_vehicle_data[i].in_int = false;
        m_vehicle_data[i].int_entrance = -1;

        #ifdef EMS_OVERRIDE_ENABLE
        m_vehicle_data[i].veh_type = NORMAL;
        m_vehicle_data[i].override = false;
        #endif

        //new vehicle
        new_vehicles[i] = m_vehicle_data[i];
    }
}
#endif

#ifdef EMS_OVERRIDE_ENABLE
void intersection::init_ems_override(void)
{
    num_overrides = 0;

    ems_override_active = false;
    ems_override_active_id = 0;
    ems_override_entrance = 0;

    for (int i = 0; i < MAX_VEHICLES; i++)
    {
        ems_override_queue[i] = 0;
    }
}
#endif

#ifdef STOP_SIGN_ENABLE
void intersection::init_stop_sign(void)
{
    num_stopped_vehicles = 0;

    next_stopped_veh_id = 0; //used before accounting for EMS
    c_int_next_veh_id = 0; //actual output
    
    for (int i = 0; i < MAX_VEHICLES; i++)
    {
        stopped_vehicles[i].veh_id = 0;
        stopped_vehicles[i].time_stopped = 0;
    }
}
#endif

void intersection::init_traffic_signal(void)
{
    //Intersection Direction #1 will be GREEN
    c_int_1_state = GREEN;
    c_int_1_next_state = YELLOW;
    c_int_1_time_to_switch = p_time_to_switch_1[0];
    c_int_1_ped_countdown = p_ped_countdowns;

    //Intersection Direction #2 will be RED
    c_int_2_state = RED;
    c_int_2_next_state = GREEN;
    c_int_2_time_to_switch = p_time_to_switch_2[2];
    c_int_2_ped_countdown = 0; //0 since red light = no crossing
}

void intersection::init_states(void)
{
    if (p_time_to_switch_1[1] < MIN_YELLOW)
    {
        std::cout << "Yellow light time below minimum!\nSetting to minimum time...\n";
        p_time_to_switch_1[1] = MIN_YELLOW;
    }

    //now set dir 2 times (preliminary)
    p_time_to_switch_2[0] = p_time_to_switch_1[2] - p_time_to_switch_1[1]; //GREEN = RED - YELLOW
    p_time_to_switch_2[1] = p_time_to_switch_1[1]; //YELLOW (share same time)
    p_time_to_switch_2[2] = p_time_to_switch_1[0] + p_time_to_switch_1[1]; //RED = GREEN + YELLOW in opposite direction

    //lastly check ped countdowns are valid
    if (p_ped_countdowns > p_time_to_switch_1[0] || p_ped_countdowns > p_time_to_switch_2[0])
    {
        //pedestrian countdowns longer than intial time to switch, set ped countdwons to time to switch (minimum)
        std::cout << "Pedestrian countdown larger than starting GREEN time to switch!\nSetting pedestrian countdown to minimum GREEN time to switch...\n";
        p_ped_countdowns = fmin(p_time_to_switch_1[0], p_time_to_switch_2[0]);
    }
}

void intersection::init_int_ent_data(void)
{
    //entrance directions are in degrees
    //entrance 0 is forward from center by dir 1 (heading is opposite of dir 1)
    //entrance 1 is backward from center by dir 1 (heading is same as dir 1)
    //entrance 2 is forward from center by dir 2 (heading is opposite of dir 2)
    //entrance 3 is backward from center by dir 2 (heading is same as dir 2)
    //all the entrance headings are backwards of these since those are angle of entrance through intersection
    double x, y, angle;
    
    for (int i = 0; i < 4; i++)
    {
        if (i < 2) //Entrance Direction #1
        {   
            angle = k_ent_dirs[0]*M_PI/180;
        }
        else //Entrance Direction #2
        {
            angle = k_ent_dirs[1]*M_PI/180;
        }
        
        double angle_2 = angle + M_PI/180*90;
        double x_delta = k_int_length/2*cos(angle) + k_int_length/4*cos(angle_2); //x is forward, angle is CCW from North
        double y_delta = k_int_length/2*sin(angle) + k_int_length/4*sin(angle_2); //y is side, angle is CCW from North

        if (i == 0 || i == 2)
        {
            //0 deg
            x = k_int_pos_x + x_delta;
            y = k_int_pos_y + y_delta;
        }
        else
        {
            //180 deg
            x = k_int_pos_x - x_delta;
            y = k_int_pos_y - y_delta;
        }

        if (i == 0)
        {
            //need to initialize values
            min_x = x;
            max_x = x;
            min_y = y;
            max_y = y;
        }
        else
        {
            //compare
            min_x = fmin(min_x, x);
            max_x = fmax(max_x, x);
            min_y = fmin(min_y, y);
            max_y = fmax(max_y, y);
        }

        int_ent_pos_x[i] = x;
        int_ent_pos_y[i] = y;

        #ifdef DEBUG
        std::cout << "Ent #: " << i << "\tX: " << x << "\tY: " << y << "\n";
        #endif

    }

    #ifdef DEBUG
    std::cout << "Min X: " << min_x << "\tMax X: " << max_x << "\tMin Y: " << min_y << "\tMax Y: " << max_y << "\n";
    #endif

    //setup entrance headings
    if (k_ent_dirs[0] >= 180)
    {
        int_ent_headings[0] = k_ent_dirs[0] - 180;
    }
    else 
    {
        int_ent_headings[0] = k_ent_dirs[0] + 180;
    }

    int_ent_headings[1] = k_ent_dirs[0];

    if (k_ent_dirs[1] >= 180)
    {
        int_ent_headings[2] = k_ent_dirs[1] - 180;
    }
    else 
    {
        int_ent_headings[2] = k_ent_dirs[1] + 180;
    }

    int_ent_headings[3] = k_ent_dirs[1];
}