// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vehicle_main:msg/IntersectionMsg.idl
// generated code does not contain a copyright notice

#ifndef VEHICLE_MAIN__MSG__DETAIL__INTERSECTION_MSG__STRUCT_H_
#define VEHICLE_MAIN__MSG__DETAIL__INTERSECTION_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/IntersectionMsg in the package vehicle_main.
typedef struct vehicle_main__msg__IntersectionMsg
{
  uint32_t position_x;
  uint32_t position_y;
  uint8_t num_directions;
  uint32_t directions[32];
  uint8_t intersection_state;
  uint8_t intersection_next_state;
  double intersection_switch_time;
} vehicle_main__msg__IntersectionMsg;

// Struct for a sequence of vehicle_main__msg__IntersectionMsg.
typedef struct vehicle_main__msg__IntersectionMsg__Sequence
{
  vehicle_main__msg__IntersectionMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vehicle_main__msg__IntersectionMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VEHICLE_MAIN__MSG__DETAIL__INTERSECTION_MSG__STRUCT_H_
