//intersection.hpp
#include "defines.hpp"
#include "common.hpp"

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

//Class
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
        void new_vehicle_callback(s_published_vehicle_data data);
        #else
        void new_vehicle_callback(const std_msgs::String::ConstPtr& msg);
        #endif

        //Publish
        #ifdef SIM
        s_published_int_data publish(void);
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
        s_input_vehicle_data m_vehicle_data[MAX_VEHICLES];
        int num_new_vehicles;
        s_input_vehicle_data new_vehicles[MAX_VEHICLES];
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
        double int_ent_headings[4];
        double min_x;
        double max_x;
        double min_y;
        double max_y;

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
        void init_int_ent_data(void);
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
        void reset_new_vehicles(void);
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