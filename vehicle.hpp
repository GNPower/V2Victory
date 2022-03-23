//vehicle.hpp
#include "defines.hpp"
#include "common.hpp"

//Structs
struct s_vehicle_data {
    unsigned int veh_id;
    double veh_pos_x;
    double veh_pos_y;
    double veh_speed;
    double veh_heading;

    double time_since_update;

    #ifdef PLATOONING_ENABLE
    bool platooning;
    double platoon_set_speed;
    #endif
};

struct s_int_data {
    double int_ent_pos_x[4];
    double int_ent_pos_y[4];
    double int_ent_headings[4];
    double int_length;

    unsigned int int_id;
    double time_since_update;

    y_int_state int_1_state;
    y_int_state int_1_next_state;
    double int_1_time_to_switch;

    y_int_state int_2_state;
    y_int_state int_2_next_state;
    double int_2_time_to_switch;

    #ifdef STOP_SIGN_ENABLE
    y_int_type int_type;
    unsigned int int_next_veh_id;
    #endif
};


//Class
class vehicle
{
    public:
        //Initialization
        #ifdef EMS_OVERRIDE_ENABLE
        //need to account for k_veh_type
        #ifdef PLATOONING_ENABLE
        //Platooning + EMS
        vehicle(unsigned int id, double max_speed, y_veh_type veh_type, double front_pos, double set_speed, double pos_x, double pos_y, double speed, double heading, bool override, bool platooning, double driver_accel_req);
        #else
        //EMS
        vehicle(unsigned int id, double max_speed, y_veh_type veh_type, double front_pos, double set_speed, double pos_x, double pos_y, double speed, double heading, bool override, double driver_accel_req);
        #endif
        #else
        #ifdef PLATOONING_ENABLE
        //Platooning
        vehicle(unsigned int id, double max_speed, double front_pos, double set_speed, double pos_x, double pos_y, double speed, double heading, bool platooning, double driver_accel_req);
        #else
        //None
        vehicle(unsigned int id, double max_speed, double front_pos, double set_speed, double pos_x, double pos_y, double speed, double heading, double driver_accel_req);
        #endif
        #endif

        //Update
        void update(double dt);

        //Callback
        #ifdef SIM
        void new_vehicle_callback(s_published_vehicle_data data);
        void new_int_callback(s_published_int_data data);
        #else
        void new_vehicle_callback(const std_msgs::String::ConstPtr& msg);
        void new_int_callback(const std_msgs::String::ConstPtr& msg);
        #endif

        void ego_data_callback(double pos_x, double pos_y, double speed, double heading);
        void driver_req_callback(double driver_accel_req);
        void set_speed_callback(double set_speed);

        #ifdef EMS_OVERRIDE_ENABLE
        void override_callback(bool override);
        #endif
        
        #ifdef PLATOONING_ENABLE
        void platooning_callback(bool platooning);
        #endif

        //Publish
        #ifdef SIM
        s_published_vehicle_data publish(void);
        #else
        void publish(void);
        #endif

        //Other Outputs
        double get_accel_req(void); //for updating dynamics model
        bool get_driver_alerts(void); //can be switched to string later if wanted

    private:
        //Constants
        unsigned int k_id;
        double k_max_speed;
        double k_front_pos;

        #ifdef EMS_OVERRIDE_ENABLE
        y_veh_type k_veh_type;
        #endif

        //Parameters
        double p_set_speed;
        
        #ifdef EMS_OVERRIDE_ENABLE
        bool p_override;
        #endif

        #ifdef PLATOONING_ENABLE
        bool p_platooning;
        #endif

        //Monitored Variables
        //Ego Vehicle Stuff from GPS & IMU
        double m_ego_pos_x;
        double m_ego_pos_y;
        double m_ego_speed;
        double m_ego_heading;
        bool ego_updated;

        //Driver Accel Request
        double m_driver_accel_req;

        //Other Vehicles
        int num_vehicles;
        s_vehicle_data m_vehicle_data[MAX_VEHICLES];
        int num_new_vehicles;
        s_vehicle_data new_vehicles[MAX_VEHICLES];

        //Intersections
        int num_ints;
        s_int_data m_int_data[MAX_INTS];
        int num_new_ints;
        s_int_data new_ints[MAX_INTS];

        //Controlled Variables
        double c_accel_req;
        bool c_driver_alert;

        #ifdef PLATOONING_ENABLE
        double c_platoon_set_speed;
        #endif

        //Internal Variables
        //Coasted Ego Data
        double coasted_ego_pos_x;
        double coasted_ego_pos_y;
        double coasted_ego_speed;

        //Lead Vehicle (Vehicle Proximity Logic)
        bool lead_exists;
        double lead_rel_pos_x;
        double lead_rel_vel_x;

        #ifdef PLATOONING_ENABLE
        bool lead_platooning;
        double lead_platoon_set_speed;
        #endif

        //Relevant Intersection
        bool approaching_int;
        double int_ent_dist;
        double int_exit_dist;
        y_int_state int_state;
        y_int_state int_next_state;
        double int_time_to_switch;

        #ifdef STOP_SIGN_ENABLE
        y_int_type int_type;
        unsigned int int_next_id;
        #endif

        //Acceleration Decision Logic
        double system_accel_req;

        //Functions
        //Init
        void validate_set_speed(void);
        void init_veh_data(void);
        void init_int_data(void);
        void init_outputs(void);
        void init_lead_veh(void);
        void init_rel_int(void);

        //Update
        void update_veh_data(double dt);
        void reset_new_veh(void);
        void update_int_data(double dt);
        void reset_new_int(void);
        void update_ego_data(double dt);
        void update_lead_veh(void);
        void update_rel_int(void);
        void update_accel_req(void);
        void update_sys_accel_req(void);
        double get_int_accel_req(void);
        double get_lead_accel_req(void);
        #ifdef PLATOONING_ENABLE
        void update_platooning(void);
        #endif
};