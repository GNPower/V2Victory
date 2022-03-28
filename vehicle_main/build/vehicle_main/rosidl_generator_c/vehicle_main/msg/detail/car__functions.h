// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from vehicle_main:msg/Car.idl
// generated code does not contain a copyright notice

#ifndef VEHICLE_MAIN__MSG__DETAIL__CAR__FUNCTIONS_H_
#define VEHICLE_MAIN__MSG__DETAIL__CAR__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "vehicle_main/msg/rosidl_generator_c__visibility_control.h"

#include "vehicle_main/msg/detail/car__struct.h"

/// Initialize msg/Car message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * vehicle_main__msg__Car
 * )) before or use
 * vehicle_main__msg__Car__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_vehicle_main
bool
vehicle_main__msg__Car__init(vehicle_main__msg__Car * msg);

/// Finalize msg/Car message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_vehicle_main
void
vehicle_main__msg__Car__fini(vehicle_main__msg__Car * msg);

/// Create msg/Car message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * vehicle_main__msg__Car__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_vehicle_main
vehicle_main__msg__Car *
vehicle_main__msg__Car__create();

/// Destroy msg/Car message.
/**
 * It calls
 * vehicle_main__msg__Car__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_vehicle_main
void
vehicle_main__msg__Car__destroy(vehicle_main__msg__Car * msg);


/// Initialize array of msg/Car messages.
/**
 * It allocates the memory for the number of elements and calls
 * vehicle_main__msg__Car__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_vehicle_main
bool
vehicle_main__msg__Car__Sequence__init(vehicle_main__msg__Car__Sequence * array, size_t size);

/// Finalize array of msg/Car messages.
/**
 * It calls
 * vehicle_main__msg__Car__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_vehicle_main
void
vehicle_main__msg__Car__Sequence__fini(vehicle_main__msg__Car__Sequence * array);

/// Create array of msg/Car messages.
/**
 * It allocates the memory for the array and calls
 * vehicle_main__msg__Car__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_vehicle_main
vehicle_main__msg__Car__Sequence *
vehicle_main__msg__Car__Sequence__create(size_t size);

/// Destroy array of msg/Car messages.
/**
 * It calls
 * vehicle_main__msg__Car__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_vehicle_main
void
vehicle_main__msg__Car__Sequence__destroy(vehicle_main__msg__Car__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // VEHICLE_MAIN__MSG__DETAIL__CAR__FUNCTIONS_H_
