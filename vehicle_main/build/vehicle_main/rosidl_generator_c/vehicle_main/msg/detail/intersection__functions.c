// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vehicle_main:msg/Intersection.idl
// generated code does not contain a copyright notice
#include "vehicle_main/msg/detail/intersection__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
vehicle_main__msg__Intersection__init(vehicle_main__msg__Intersection * msg)
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
vehicle_main__msg__Intersection__fini(vehicle_main__msg__Intersection * msg)
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

vehicle_main__msg__Intersection *
vehicle_main__msg__Intersection__create()
{
  vehicle_main__msg__Intersection * msg = (vehicle_main__msg__Intersection *)malloc(sizeof(vehicle_main__msg__Intersection));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vehicle_main__msg__Intersection));
  bool success = vehicle_main__msg__Intersection__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
vehicle_main__msg__Intersection__destroy(vehicle_main__msg__Intersection * msg)
{
  if (msg) {
    vehicle_main__msg__Intersection__fini(msg);
  }
  free(msg);
}


bool
vehicle_main__msg__Intersection__Sequence__init(vehicle_main__msg__Intersection__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  vehicle_main__msg__Intersection * data = NULL;
  if (size) {
    data = (vehicle_main__msg__Intersection *)calloc(size, sizeof(vehicle_main__msg__Intersection));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vehicle_main__msg__Intersection__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vehicle_main__msg__Intersection__fini(&data[i - 1]);
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
vehicle_main__msg__Intersection__Sequence__fini(vehicle_main__msg__Intersection__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      vehicle_main__msg__Intersection__fini(&array->data[i]);
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

vehicle_main__msg__Intersection__Sequence *
vehicle_main__msg__Intersection__Sequence__create(size_t size)
{
  vehicle_main__msg__Intersection__Sequence * array = (vehicle_main__msg__Intersection__Sequence *)malloc(sizeof(vehicle_main__msg__Intersection__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = vehicle_main__msg__Intersection__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
vehicle_main__msg__Intersection__Sequence__destroy(vehicle_main__msg__Intersection__Sequence * array)
{
  if (array) {
    vehicle_main__msg__Intersection__Sequence__fini(array);
  }
  free(array);
}
