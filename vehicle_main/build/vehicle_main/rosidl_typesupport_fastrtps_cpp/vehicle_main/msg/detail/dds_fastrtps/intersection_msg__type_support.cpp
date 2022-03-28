// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from vehicle_main:msg/IntersectionMsg.idl
// generated code does not contain a copyright notice
#include "vehicle_main/msg/detail/intersection_msg__rosidl_typesupport_fastrtps_cpp.hpp"
#include "vehicle_main/msg/detail/intersection_msg__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace vehicle_main
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vehicle_main
cdr_serialize(
  const vehicle_main::msg::IntersectionMsg & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: position_x
  cdr << ros_message.position_x;
  // Member: position_y
  cdr << ros_message.position_y;
  // Member: num_directions
  cdr << ros_message.num_directions;
  // Member: directions
  {
    cdr << ros_message.directions;
  }
  // Member: intersection_state
  cdr << ros_message.intersection_state;
  // Member: intersection_next_state
  cdr << ros_message.intersection_next_state;
  // Member: intersection_switch_time
  cdr << ros_message.intersection_switch_time;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vehicle_main
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  vehicle_main::msg::IntersectionMsg & ros_message)
{
  // Member: position_x
  cdr >> ros_message.position_x;

  // Member: position_y
  cdr >> ros_message.position_y;

  // Member: num_directions
  cdr >> ros_message.num_directions;

  // Member: directions
  {
    cdr >> ros_message.directions;
  }

  // Member: intersection_state
  cdr >> ros_message.intersection_state;

  // Member: intersection_next_state
  cdr >> ros_message.intersection_next_state;

  // Member: intersection_switch_time
  cdr >> ros_message.intersection_switch_time;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vehicle_main
get_serialized_size(
  const vehicle_main::msg::IntersectionMsg & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: position_x
  {
    size_t item_size = sizeof(ros_message.position_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: position_y
  {
    size_t item_size = sizeof(ros_message.position_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: num_directions
  {
    size_t item_size = sizeof(ros_message.num_directions);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: directions
  {
    size_t array_size = 32;
    size_t item_size = sizeof(ros_message.directions[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: intersection_state
  {
    size_t item_size = sizeof(ros_message.intersection_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: intersection_next_state
  {
    size_t item_size = sizeof(ros_message.intersection_next_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: intersection_switch_time
  {
    size_t item_size = sizeof(ros_message.intersection_switch_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vehicle_main
max_serialized_size_IntersectionMsg(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: position_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: position_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: num_directions
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: directions
  {
    size_t array_size = 32;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: intersection_state
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: intersection_next_state
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: intersection_switch_time
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  return current_alignment - initial_alignment;
}

static bool _IntersectionMsg__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const vehicle_main::msg::IntersectionMsg *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _IntersectionMsg__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<vehicle_main::msg::IntersectionMsg *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _IntersectionMsg__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const vehicle_main::msg::IntersectionMsg *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _IntersectionMsg__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_IntersectionMsg(full_bounded, 0);
}

static message_type_support_callbacks_t _IntersectionMsg__callbacks = {
  "vehicle_main::msg",
  "IntersectionMsg",
  _IntersectionMsg__cdr_serialize,
  _IntersectionMsg__cdr_deserialize,
  _IntersectionMsg__get_serialized_size,
  _IntersectionMsg__max_serialized_size
};

static rosidl_message_type_support_t _IntersectionMsg__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_IntersectionMsg__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace vehicle_main

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_vehicle_main
const rosidl_message_type_support_t *
get_message_type_support_handle<vehicle_main::msg::IntersectionMsg>()
{
  return &vehicle_main::msg::typesupport_fastrtps_cpp::_IntersectionMsg__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vehicle_main, msg, IntersectionMsg)() {
  return &vehicle_main::msg::typesupport_fastrtps_cpp::_IntersectionMsg__handle;
}

#ifdef __cplusplus
}
#endif
