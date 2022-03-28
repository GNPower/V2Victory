// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from vehicle_main:msg/IntersectionMsg.idl
// generated code does not contain a copyright notice
#include "vehicle_main/msg/detail/intersection_msg__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "vehicle_main/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "vehicle_main/msg/detail/intersection_msg__struct.h"
#include "vehicle_main/msg/detail/intersection_msg__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _IntersectionMsg__ros_msg_type = vehicle_main__msg__IntersectionMsg;

static bool _IntersectionMsg__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _IntersectionMsg__ros_msg_type * ros_message = static_cast<const _IntersectionMsg__ros_msg_type *>(untyped_ros_message);
  // Field name: position_x
  {
    cdr << ros_message->position_x;
  }

  // Field name: position_y
  {
    cdr << ros_message->position_y;
  }

  // Field name: num_directions
  {
    cdr << ros_message->num_directions;
  }

  // Field name: directions
  {
    size_t size = 32;
    auto array_ptr = ros_message->directions;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: intersection_state
  {
    cdr << ros_message->intersection_state;
  }

  // Field name: intersection_next_state
  {
    cdr << ros_message->intersection_next_state;
  }

  // Field name: intersection_switch_time
  {
    cdr << ros_message->intersection_switch_time;
  }

  return true;
}

static bool _IntersectionMsg__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _IntersectionMsg__ros_msg_type * ros_message = static_cast<_IntersectionMsg__ros_msg_type *>(untyped_ros_message);
  // Field name: position_x
  {
    cdr >> ros_message->position_x;
  }

  // Field name: position_y
  {
    cdr >> ros_message->position_y;
  }

  // Field name: num_directions
  {
    cdr >> ros_message->num_directions;
  }

  // Field name: directions
  {
    size_t size = 32;
    auto array_ptr = ros_message->directions;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: intersection_state
  {
    cdr >> ros_message->intersection_state;
  }

  // Field name: intersection_next_state
  {
    cdr >> ros_message->intersection_next_state;
  }

  // Field name: intersection_switch_time
  {
    cdr >> ros_message->intersection_switch_time;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vehicle_main
size_t get_serialized_size_vehicle_main__msg__IntersectionMsg(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _IntersectionMsg__ros_msg_type * ros_message = static_cast<const _IntersectionMsg__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name position_x
  {
    size_t item_size = sizeof(ros_message->position_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name position_y
  {
    size_t item_size = sizeof(ros_message->position_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name num_directions
  {
    size_t item_size = sizeof(ros_message->num_directions);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name directions
  {
    size_t array_size = 32;
    auto array_ptr = ros_message->directions;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name intersection_state
  {
    size_t item_size = sizeof(ros_message->intersection_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name intersection_next_state
  {
    size_t item_size = sizeof(ros_message->intersection_next_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name intersection_switch_time
  {
    size_t item_size = sizeof(ros_message->intersection_switch_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _IntersectionMsg__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_vehicle_main__msg__IntersectionMsg(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vehicle_main
size_t max_serialized_size_vehicle_main__msg__IntersectionMsg(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: position_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: position_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: num_directions
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: directions
  {
    size_t array_size = 32;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: intersection_state
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: intersection_next_state
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: intersection_switch_time
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _IntersectionMsg__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_vehicle_main__msg__IntersectionMsg(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_IntersectionMsg = {
  "vehicle_main::msg",
  "IntersectionMsg",
  _IntersectionMsg__cdr_serialize,
  _IntersectionMsg__cdr_deserialize,
  _IntersectionMsg__get_serialized_size,
  _IntersectionMsg__max_serialized_size
};

static rosidl_message_type_support_t _IntersectionMsg__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_IntersectionMsg,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, vehicle_main, msg, IntersectionMsg)() {
  return &_IntersectionMsg__type_support;
}

#if defined(__cplusplus)
}
#endif
