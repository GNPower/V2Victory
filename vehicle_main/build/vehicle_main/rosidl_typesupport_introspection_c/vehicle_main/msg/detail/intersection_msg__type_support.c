// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from vehicle_main:msg/IntersectionMsg.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "vehicle_main/msg/detail/intersection_msg__rosidl_typesupport_introspection_c.h"
#include "vehicle_main/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "vehicle_main/msg/detail/intersection_msg__functions.h"
#include "vehicle_main/msg/detail/intersection_msg__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void IntersectionMsg__rosidl_typesupport_introspection_c__IntersectionMsg_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  vehicle_main__msg__IntersectionMsg__init(message_memory);
}

void IntersectionMsg__rosidl_typesupport_introspection_c__IntersectionMsg_fini_function(void * message_memory)
{
  vehicle_main__msg__IntersectionMsg__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember IntersectionMsg__rosidl_typesupport_introspection_c__IntersectionMsg_message_member_array[7] = {
  {
    "position_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vehicle_main__msg__IntersectionMsg, position_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vehicle_main__msg__IntersectionMsg, position_y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "num_directions",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vehicle_main__msg__IntersectionMsg, num_directions),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "directions",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    32,  // array size
    false,  // is upper bound
    offsetof(vehicle_main__msg__IntersectionMsg, directions),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "intersection_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vehicle_main__msg__IntersectionMsg, intersection_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "intersection_next_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vehicle_main__msg__IntersectionMsg, intersection_next_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "intersection_switch_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vehicle_main__msg__IntersectionMsg, intersection_switch_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers IntersectionMsg__rosidl_typesupport_introspection_c__IntersectionMsg_message_members = {
  "vehicle_main__msg",  // message namespace
  "IntersectionMsg",  // message name
  7,  // number of fields
  sizeof(vehicle_main__msg__IntersectionMsg),
  IntersectionMsg__rosidl_typesupport_introspection_c__IntersectionMsg_message_member_array,  // message members
  IntersectionMsg__rosidl_typesupport_introspection_c__IntersectionMsg_init_function,  // function to initialize message memory (memory has to be allocated)
  IntersectionMsg__rosidl_typesupport_introspection_c__IntersectionMsg_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t IntersectionMsg__rosidl_typesupport_introspection_c__IntersectionMsg_message_type_support_handle = {
  0,
  &IntersectionMsg__rosidl_typesupport_introspection_c__IntersectionMsg_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vehicle_main
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vehicle_main, msg, IntersectionMsg)() {
  if (!IntersectionMsg__rosidl_typesupport_introspection_c__IntersectionMsg_message_type_support_handle.typesupport_identifier) {
    IntersectionMsg__rosidl_typesupport_introspection_c__IntersectionMsg_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &IntersectionMsg__rosidl_typesupport_introspection_c__IntersectionMsg_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
