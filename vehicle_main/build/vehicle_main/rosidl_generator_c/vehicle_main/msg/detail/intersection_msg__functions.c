// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vehicle_main:msg/IntersectionMsg.idl
// generated code does not contain a copyright notice
#include "vehicle_main/msg/detail/intersection_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
vehicle_main__msg__IntersectionMsg__init(vehicle_main__msg__IntersectionMsg * msg)
{
  if (!msg) {
    return false;
  }
  // position_x
  // position_y
  // num_directions
  // directions
  // intersection_state
  // intersection_next_state
  // intersection_switch_time
  return true;
}

void
vehicle_main__msg__IntersectionMsg__fini(vehicle_main__msg__IntersectionMsg * msg)
{
  if (!msg) {
    return;
  }
  // position_x
  // position_y
  // num_directions
  // directions
  // intersection_state
  // intersection_next_state
  // intersection_switch_time
}

vehicle_main__msg__IntersectionMsg *
vehicle_main__msg__IntersectionMsg__create()
{
  vehicle_main__msg__IntersectionMsg * msg = (vehicle_main__msg__IntersectionMsg *)malloc(sizeof(vehicle_main__msg__IntersectionMsg));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vehicle_main__msg__IntersectionMsg));
  bool success = vehicle_main__msg__IntersectionMsg__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
vehicle_main__msg__IntersectionMsg__destroy(vehicle_main__msg__IntersectionMsg * msg)
{
  if (msg) {
    vehicle_main__msg__IntersectionMsg__fini(msg);
  }
  free(msg);
}


bool
vehicle_main__msg__IntersectionMsg__Sequence__init(vehicle_main__msg__IntersectionMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  vehicle_main__msg__IntersectionMsg * data = NULL;
  if (size) {
    data = (vehicle_main__msg__IntersectionMsg *)calloc(size, sizeof(vehicle_main__msg__IntersectionMsg));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vehicle_main__msg__IntersectionMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vehicle_main__msg__IntersectionMsg__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
vehicle_main__msg__IntersectionMsg__Sequence__fini(vehicle_main__msg__IntersectionMsg__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      vehicle_main__msg__IntersectionMsg__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

vehicle_main__msg__IntersectionMsg__Sequence *
vehicle_main__msg__IntersectionMsg__Sequence__create(size_t size)
{
  vehicle_main__msg__IntersectionMsg__Sequence * array = (vehicle_main__msg__IntersectionMsg__Sequence *)malloc(sizeof(vehicle_main__msg__IntersectionMsg__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = vehicle_main__msg__IntersectionMsg__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
vehicle_main__msg__IntersectionMsg__Sequence__destroy(vehicle_main__msg__IntersectionMsg__Sequence * array)
{
  if (array) {
    vehicle_main__msg__IntersectionMsg__Sequence__fini(array);
  }
  free(array);
}
