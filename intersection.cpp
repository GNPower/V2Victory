#include <math.h>
#include <cmath>
#include <iostream>

//Defines
//Assumptions
#define MIN_YELLOW 3.7
#define MAX_VEHICLES 32

//Environment (SIM vs MODEL)
#define SIM

//Performance
#define MAX_COASTING 1.0 //amount of time a vehicle can be coasted before removed (coasted means no state updates received)
#define INLANE_HEADING 45.0 //angle diff between lane and vehicle that counts as being in same direction
#define TRAFFIC_ABS_DIFF 10 //amount of cars difference to trigger switch
#define TRAFFIC_REL_DIFF 4.0 //relative difference in amount of cars to trigger switch
#define MAX_STOP_SIGN_COAST 0.1 //how long since last update at stop sign
#define STOP_SIGN_DISTANCE 2.0 //range vehicle must be from entrance to be considered stopped at the intersection
#define EMS_COASTING 0.1 //how long since last update for EMS override

//Stretch Goals
#define SMART_TIMING_ENABLE //Stretch Goal #1: Smart Signal Timing
#define EMS_OVERRIDE_ENABLE //Stretch Goal #2: Emergency Vehicle Response
#define STOP_SIGN_ENABLE //Stretch Goal #4: Stop Sign Intersections

//Enums
enum y_int_state {RED, YELLOW, GREEN};
#ifdef STOP_SIGN_ENABLE
enum y_int_type {SIGNAL, STOP};
#endif
#ifdef EMS_OVERRIDE_ENABLE
enum y_veh_type {NORMAL, EMERGENCY};
#endif

//Structs
#ifdef EMS_OVERRIDE_ENABLE
struct s_override_vehicle {
    unsigned int veh_id;
    double int_dist;
};
#endif

#ifdef STOP_SIGN_ENABLE
struct s_stopped_vehicle {
    unsigned int veh_id;
    double time_stopped;
};
#endif

struct s_new_vehicle_data {
    unsigned int veh_id;
    double veh_pos_x;
    double veh_pos_y;
    double veh_speed;
    double veh_heading;

    #ifdef EMS_OVERRIDE_ENABLE
    y_veh_type veh_type;
    bool override;
    #endif
};

struct s_input_vehicle_data {
    unsigned int veh_id;
    double veh_pos_x;
    double veh_pos_y;
    double veh_speed;
    double veh_heading;
    double time_since_update;
    bool in_int;
    int int_entrance;

    #ifdef EMS_OVERRIDE_ENABLE
    y_veh_type veh_type;
    bool override;
    #endif
};

struct s_int_data {
    double ent_dir_1;
    double ent_dir_2;
    double int_length;
    double int_pos_x;
    double int_pos_y;
    unsigned int int_id;

    #ifdef STOP_SIGN_ENABLE
    y_int_type int_type;
    #endif

    y_int_state int_1_state;
    y_int_state int_1_next_state;
    double int_1_time_to_switch;

    y_int_state int_2_state;
    y_int_state int_2_next_state;
    double int_2_time_to_switch;

    #ifdef STOP_SIGN_ENABLE
    unsigned int int_next_veh_id;
    #endif
};

class intersection
{
    public:
        //Initialization
        #ifdef STOP_SIGN_ENABLE
        //need to account for k_int_type
        intersection(double ent_dirs[2], double int_length, double int_pos_x, double int_pos_y, y_int_type k_int_type, unsigned int id, double ped_countdowns, double time_to_switch[3]);
        #else
        intersection(double ent_dirs[2], double int_length, double int_pos_x, double int_pos_y, unsigned int id, double ped_countdowns, double time_to_switch[3]);
        #endif

        //Update
        void update(double dt);
        
        //Callback
        #ifdef SIM
        void new_vehicle_callback(s_new_vehicle_data data);
        #else
        void new_vehicle_callback(const std_msgs::String::ConstPtr& msg);
        #endif

        //Publish
        #ifdef SIM
        s_int_data publish(void);
        #else
        void publish(void);
        #endif

    private:
        //Constants
        double k_ent_dirs[2];
        double k_int_length;
        double k_int_pos_x;
        double k_int_pos_y;
        unsigned int k_id;

        #ifdef STOP_SIGN_ENABLE
        y_int_type k_int_type;
        #endif

        //Parameters
        y_int_state p_states[3] = {GREEN, YELLOW, RED}; //always this order since left to right is order of transitions;
        double p_ped_countdowns;
        double p_time_to_switch_1[3];
        double p_time_to_switch_2[3];

        //Monitored Variables
        #if defined(SMART_TIMING_ENABLE) || defined(STOP_SIGN_ENABLE) || defined(EMS_OVERRIDE_ENABLE)
        int num_vehicles;
        struct s_input_vehicle_data m_vehicle_data[MAX_VEHICLES];
        int num_new_vehicles;
        struct s_input_vehicle_data new_vehicles[MAX_VEHICLES];
        //new_vehicles will be updated using callback in ROS on subscriber, in sim can then use as public function to update new_vehicles
        #endif

        //Controlled Variables
        //Entrance #1
        y_int_state c_int_1_state;
        y_int_state c_int_1_next_state;
        double c_int_1_time_to_switch;
        double c_int_1_ped_countdown;
        
        //Entrance #2
        y_int_state c_int_2_state;
        y_int_state c_int_2_next_state;
        double c_int_2_time_to_switch;
        double c_int_2_ped_countdown;

        #ifdef STOP_SIGN_ENABLE
        unsigned int c_int_next_veh_id;
        #endif

        //Internal Variables
        double int_ent_pos_x[4];
        double int_ent_pos_y[4];

        #ifdef SMART_TIMING_ENABLE
        bool priority_enable;
        int priority_direction;
        #endif

        #ifdef STOP_SIGN_ENABLE
        int num_stopped_vehicles;
        s_stopped_vehicle stopped_vehicles[MAX_VEHICLES];

        unsigned int next_stopped_veh_id;
        #endif

        #ifdef EMS_OVERRIDE_ENABLE
        int num_overrides;
        unsigned int ems_override_queue[MAX_VEHICLES];
        
        bool ems_override_active;
        unsigned int ems_override_active_id;
        int ems_override_entrance;
        #endif

        //Functions
        //Init
        void init_int_ent_pos(void);
        void init_states(void);
        void init_traffic_signal(void);
        #if defined(SMART_TIMING_ENABLE) || defined(STOP_SIGN_ENABLE) || defined(EMS_OVERRIDE_ENABLE)
        void init_traffic_tracker(void);
        #endif
        #ifdef STOP_SIGN_ENABLE
        void init_stop_sign(void);
        #endif
        #ifdef EMS_OVERRIDE_ENABLE
        void init_ems_override(void);
        #endif
        

        //Update
        void update_traffic_signal(double dt);
        #if defined(SMART_TIMING_ENABLE) || defined(STOP_SIGN_ENABLE) || defined(EMS_OVERRIDE_ENABLE)
        void update_traffic_tracker(double dt);
        void assign_int_ents(void);
        #endif
        #ifdef SMART_TIMING_ENABLE
        void update_traffic_priority(void);
        #endif
        #ifdef STOP_SIGN_ENABLE
        void update_stop_sign(double dt);
        void update_stop_sign_priority(double dt);
        #endif
        #ifdef EMS_OVERRIDE_ENABLE
        void update_ems_override(double dt);
        #endif
};

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
    init_int_ent_pos();

    //States need validation
    p_ped_countdowns = ped_countdowns;
    p_time_to_switch_1[0] = time_to_switch[0];
    p_time_to_switch_1[1] = time_to_switch[1];
    p_time_to_switch_1[2] = time_to_switch[2];

    //Init Intersection
    #ifdef STOP_SIGN_ENABLE
    if (k_int_type == STOP)
    {
        init_stop_sign();
    }
    else
    {
        init_states();
        init_traffic_signal();
    }
    #else
    init_states();
    init_traffic_signal();
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
s_int_data intersection::publish(void)
#else
void intersection::publish(void)
#endif
{
    #ifdef SIM
    //just output the data
    s_int_data int_data;
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
void intersection::new_vehicle_callback(s_new_vehicle_data data)
#else
void intersection::new_vehicle_callback(const std_msgs::String::ConstPtr& msg)
#endif
{
    #ifndef SIM
    //actually parse data, put into new_vehicle_data struct
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
       } 
       else
       {
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
            //Direction #1
            //need to account for difference
            double time_diff = c_int_2_time_to_switch - c_int_2_ped_countdown;
            c_int_2_time_to_switch = c_int_2_ped_countdown;
            c_int_1_time_to_switch -= time_diff;
        }
        else if (priority_direction == 1 && c_int_2_state == RED && c_int_1_state == GREEN && c_int_1_time_to_switch > c_int_1_ped_countdown)
        {
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
                    m_vehicle_data[j] = m_vehicle_data[j+1];
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
        
        //Distance to entrance
        double ent_dist_x = coasted_x - int_ent_pos_x[0];
        double ent_dist_y = coasted_y - int_ent_pos_y[0];
        double ent_dist = pow(ent_dist_x,2) + pow(ent_dist_y,2); //don't need sqrt to determine closest
        double closest_ent_dist = ent_dist;

        //Difference in heading compared to entrance
        double dir_angle;
        if (k_ent_dirs[0] >= 180)
        {
            dir_angle = k_ent_dirs[0] - 180;
        }
        else 
        {
            dir_angle = k_ent_dirs[0] + 180;
        }
        double ent_heading = abs(dir_angle - m_vehicle_data[i].veh_heading);
        if (ent_heading > 180)
        {
            ent_heading = 360 - ent_heading;
        }
        double closest_ent_heading = ent_heading;

        for (int j = 1; j < 4; j++) //start at 1 since 0 index used in init
        {
            //Calculate Distance
            ent_dist_x = coasted_x - int_ent_pos_x[j];
            ent_dist_y = coasted_y - int_ent_pos_y[j];
            ent_dist = pow(ent_dist_x, 2) + pow(ent_dist_y, 2);

            //Calculate Heading
            switch(j)
            {
                case 1:
                    dir_angle = k_ent_dirs[0];
                    break;
                case 2:
                    if (k_ent_dirs[1] >= 180)
                    {
                        dir_angle = k_ent_dirs[1] - 180;
                    }
                    else 
                    {
                        dir_angle = k_ent_dirs[1] + 180;
                    }
                    break;
                case 3:
                    dir_angle = k_ent_dirs[1];
                    break;
            }
            ent_heading = abs(dir_angle - m_vehicle_data[i].veh_heading);
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
            }
        }
        //now have closest int, distance to int, heading difference

        //check in intersection:
        //get min and max x and y (needs to be within bounds)
        bool in_int;
        double min_x, max_x, min_y, max_y;
        if (closest_ent == 0 || closest_ent == 1)
        {
            min_x = fmin(int_ent_pos_x[0], int_ent_pos_x[1]);
            max_x = fmax(int_ent_pos_x[0], int_ent_pos_x[1]);
            min_y = fmin(int_ent_pos_y[0], int_ent_pos_y[1]);
            max_y = fmax(int_ent_pos_y[0], int_ent_pos_y[1]);
        }
        else
        {
            min_x = fmin(int_ent_pos_x[2], int_ent_pos_x[3]);
            max_x = fmax(int_ent_pos_x[2], int_ent_pos_x[3]);
            min_y = fmin(int_ent_pos_y[2], int_ent_pos_y[3]);
            max_y = fmax(int_ent_pos_y[2], int_ent_pos_y[3]);
        }
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
                if (m_vehicle_data[j].veh_speed <= 0.1 && m_vehicle_data[j].time_since_update <= MAX_STOP_SIGN_COAST && m_vehicle_data[i].int_entrance != -1 && !m_vehicle_data[i].in_int)
                {
                    //still stopped, recently updated, approaching intersection, not inside intersection
                    stopped_vehicles[i].time_stopped += dt;
                }
                else
                {
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
        if (m_vehicle_data[i].veh_speed <= 0.1 && m_vehicle_data[i].time_since_update <= MAX_STOP_SIGN_COAST && m_vehicle_data[i].int_entrance != -1 && !m_vehicle_data[i].in_int)
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
    if (next_stopped_veh_id == 0)
    {
        double max_time_stopped = 0;
        for (int i = 0; i < num_stopped_vehicles; i++)
        {
            if (stopped_vehicles[i].time_stopped > max_time_stopped)
            {
                //dont bother checking position if it wont beat current max stopping time
                unsigned int stopped_id = stopped_vehicles[i].veh_id;
                for (int j = 0; j < num_vehicles; j++)
                {
                    if (stopped_id == m_vehicle_data[j].veh_id)
                    {
                        double stop_dist_x = m_vehicle_data[j].veh_pos_x - int_ent_pos_x[m_vehicle_data[j].int_entrance];
                        double stop_dist_y = m_vehicle_data[j].veh_pos_y - int_ent_pos_y[m_vehicle_data[j].int_entrance];
                        double stop_dist = sqrt(pow(stop_dist_x, 2) + pow(stop_dist_y, 2));
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
    for (int i = 0; i < MAX_VEHICLES; i++)
    {
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

void intersection::init_int_ent_pos(void)
{
    //entrance directions are in degrees
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
            
        double x_delta = k_int_length/2*cos(angle); //x is forward, angle is CCW from North
        double y_delta = k_int_length/2*sin(angle); //y is side, angle is CCW from North

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

        int_ent_pos_x[i] = x;
        int_ent_pos_y[i] = y;
    }
}

int main(void)
{
    std::cout << "Initializing...\n";
    double ent_dirs[2] = {0.0, 90.0};
    double tts[3] = {30.0, 4.0, 40.0};
    double int_length = 10.0;
    double int_pos_x = 0.0;
    double int_pos_y = 0.0;
    y_int_type int_type = SIGNAL;
    double ped_countdown = 5.0;

    intersection test_int(ent_dirs, int_length, int_pos_x, int_pos_y, int_type, 1, ped_countdown, tts);
    std::cout << "Initialized\n";
    
}