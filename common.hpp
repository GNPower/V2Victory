//common.hpp
#include "defines.hpp"

#ifndef SYS_COMMON
#define SYS_COMMON

//Enums
enum y_int_state {RED, YELLOW, GREEN};
#ifdef STOP_SIGN_ENABLE
enum y_int_type {SIGNAL, STOP};
#endif
#ifdef EMS_OVERRIDE_ENABLE
enum y_veh_type {NORMAL, EMERGENCY};
#endif

//Structs
struct s_published_vehicle_data {
    unsigned int veh_id;
    double veh_pos_x;
    double veh_pos_y;
    double veh_speed;
    double veh_heading;

    #ifdef EMS_OVERRIDE_ENABLE
    y_veh_type veh_type;
    bool override;
    #endif

    #ifdef PLATOONING_ENABLE
    bool platooning;
    double platoon_set_speed;
    #endif
};

struct s_published_int_data {
    double ent_dir_1;
    double ent_dir_2;
    double int_length;
    double int_pos_x;
    double int_pos_y;
    unsigned int int_id;

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

#endif