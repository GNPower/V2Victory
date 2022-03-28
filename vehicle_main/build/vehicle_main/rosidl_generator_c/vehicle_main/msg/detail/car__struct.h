// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vehicle_main:msg/Car.idl
// generated code does not contain a copyright notice

#ifndef VEHICLE_MAIN__MSG__DETAIL__CAR__STRUCT_H_
#define VEHICLE_MAIN__MSG__DETAIL__CAR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/Car in the package vehicle_main.
typedef struct vehicle_main__msg__Car
{
  uint32_t position_x;
  uint32_t position_y;
  uint32_t heading;
  double vehicle_speed;
  uint8_t priority;
} vehicle_main__msg__Car;

// Struct for a sequence of vehicle_main__msg__Car.
typedef struct vehicle_main__msg__Car__Sequence
{
  vehicle_main__msg__Car * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vehicle_main__msg__Car__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VEHICLE_MAIN__MSG__DETAIL__CAR__STRUCT_H_
