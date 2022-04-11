#include "common.hpp"

uint8_t get_veh_crc(s_published_vehicle_data veh_data)
{
    unsigned int veh_crc_full  = veh_data.veh_id ^ (unsigned int) veh_data.veh_pos_x ^ (unsigned int) veh_data.veh_pos_y;
    veh_crc_full = veh_crc_full ^ (unsigned int) veh_data.veh_speed ^ (unsigned int) veh_data.veh_heading;
    uint8_t veh_crc = veh_crc_full & 0xFF;
    return veh_crc;
}

uint8_t get_int_crc(s_published_int_data int_data)
{
    unsigned int int_crc_full = int_data.int_id ^ (unsigned int) int_data.int_length;
    int_crc_full = int_crc_full ^ (unsigned int) int_data.ent_dir_1 ^ (unsigned int) int_data.ent_dir_2 ^ (unsigned int) int_data.int_1_time_to_switch ^ (unsigned int) int_data.int_2_time_to_switch;
    uint8_t int_crc = int_crc_full & 0xFF;
    return int_crc;
}