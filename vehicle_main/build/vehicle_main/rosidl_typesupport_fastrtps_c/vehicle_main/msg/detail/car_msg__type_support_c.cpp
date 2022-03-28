// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from vehicle_main:msg/CarMsg.idl
// generated code does not contain a copyright notice
#include "vehicle_main/msg/detail/car_msg__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "vehicle_main/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "vehicle_main/msg/detail/car_msg__struct.h"
#include "vehicle_main/msg/detail/car_msg__functions.h"
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


using _CarMsg__ros_msg_type = vehicle_main__msg__CarMsg;

static bool _CarMsg__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _CarMsg__ros_msg_type * ros_message = static_cast<const _CarMsg__ros_msg_type *>(untyped_ros_message);
  // Field name: position_x
  {
    cdr << ros_message->position_x;
  }

  // Field name: position_y
  {
    cdr << ros_message->position_y;
  }

  // Field name: heading
  {
    cdr << ros_message->heading;
  }

  // Field name: vehicle_speed
  {
    cdr << ros_message->vehicle_speed;
  }

  // Field name: priority
  {
    cdr << ros_message->priority;
  }

  return true;
}

static bool _CarMsg__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _CarMsg__ros_msg_type * ros_message = static_cast<_CarMsg__ros_msg_type *>(untyped_ros_message);
  // Field name: position_x
  {
    cdr >> ros_message->position_x;
  }

  // Field name: position_y
  {
    cdr >> ros_message->position_y;
  }

  // Field name: heading
  {
    cdr >> ros_message->heading;
  }

  // Field name: vehicle_speed
  {
    cdr >> ros_message->vehicle_speed;
  }

  // Field name: priority
  {
    cdr >> ros_message->priority;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vehicle_main
size_t get_serialized_size_vehicle_main__msg__CarMsg(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _CarMsg__ros_msg_type * ros_message = static_cast<const _CarMsg__ros_msg_type *>(untyped_ros_message);
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
  // field.name heading
  {
    size_t item_size = sizeof(ros_message->heading);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name vehicle_speed
  {
    size_t item_size = sizeof(ros_message->vehicle_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name priority
  {
    size_t item_size = sizeof(ros_message->priority);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _CarMsg__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_vehicle_main__msg__CarMsg(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vehicle_main
size_t max_serialized_size_vehicle_main__msg__CarMsg(
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
  // member: heading
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: vehicle_speed
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: priority
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _CarMsg__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_vehicle_main__msg__CarMsg(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_CarMsg = {
  "vehicle_main::msg",
  "CarMsg",
  _CarMsg__cdr_serialize,
  _CarMsg__cdr_deserialize,
  _CarMsg__get_serialized_size,
  _CarMsg__max_serialized_size
};

static rosidl_message_type_support_t _CarMsg__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_CarMsg,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, vehicle_main, msg, CarMsg)() {
  return &_CarMsg__type_support;
}

#if defined(__cplusplus)
}
#endif
