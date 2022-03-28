// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vehicle_main:msg/IntersectionMsg.idl
// generated code does not contain a copyright notice

#ifndef VEHICLE_MAIN__MSG__DETAIL__INTERSECTION_MSG__BUILDER_HPP_
#define VEHICLE_MAIN__MSG__DETAIL__INTERSECTION_MSG__BUILDER_HPP_

#include "vehicle_main/msg/detail/intersection_msg__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace vehicle_main
{

namespace msg
{

namespace builder
{

class Init_IntersectionMsg_intersection_switch_time
{
public:
  explicit Init_IntersectionMsg_intersection_switch_time(::vehicle_main::msg::IntersectionMsg & msg)
  : msg_(msg)
  {}
  ::vehicle_main::msg::IntersectionMsg intersection_switch_time(::vehicle_main::msg::IntersectionMsg::_intersection_switch_time_type arg)
  {
    msg_.intersection_switch_time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vehicle_main::msg::IntersectionMsg msg_;
};

class Init_IntersectionMsg_intersection_next_state
{
public:
  explicit Init_IntersectionMsg_intersection_next_state(::vehicle_main::msg::IntersectionMsg & msg)
  : msg_(msg)
  {}
  Init_IntersectionMsg_intersection_switch_time intersection_next_state(::vehicle_main::msg::IntersectionMsg::_intersection_next_state_type arg)
  {
    msg_.intersection_next_state = std::move(arg);
    return Init_IntersectionMsg_intersection_switch_time(msg_);
  }

private:
  ::vehicle_main::msg::IntersectionMsg msg_;
};

class Init_IntersectionMsg_intersection_state
{
public:
  explicit Init_IntersectionMsg_intersection_state(::vehicle_main::msg::IntersectionMsg & msg)
  : msg_(msg)
  {}
  Init_IntersectionMsg_intersection_next_state intersection_state(::vehicle_main::msg::IntersectionMsg::_intersection_state_type arg)
  {
    msg_.intersection_state = std::move(arg);
    return Init_IntersectionMsg_intersection_next_state(msg_);
  }

private:
  ::vehicle_main::msg::IntersectionMsg msg_;
};

class Init_IntersectionMsg_directions
{
public:
  explicit Init_IntersectionMsg_directions(::vehicle_main::msg::IntersectionMsg & msg)
  : msg_(msg)
  {}
  Init_IntersectionMsg_intersection_state directions(::vehicle_main::msg::IntersectionMsg::_directions_type arg)
  {
    msg_.directions = std::move(arg);
    return Init_IntersectionMsg_intersection_state(msg_);
  }

private:
  ::vehicle_main::msg::IntersectionMsg msg_;
};

class Init_IntersectionMsg_num_directions
{
public:
  explicit Init_IntersectionMsg_num_directions(::vehicle_main::msg::IntersectionMsg & msg)
  : msg_(msg)
  {}
  Init_IntersectionMsg_directions num_directions(::vehicle_main::msg::IntersectionMsg::_num_directions_type arg)
  {
    msg_.num_directions = std::move(arg);
    return Init_IntersectionMsg_directions(msg_);
  }

private:
  ::vehicle_main::msg::IntersectionMsg msg_;
};

class Init_IntersectionMsg_position_y
{
public:
  explicit Init_IntersectionMsg_position_y(::vehicle_main::msg::IntersectionMsg & msg)
  : msg_(msg)
  {}
  Init_IntersectionMsg_num_directions position_y(::vehicle_main::msg::IntersectionMsg::_position_y_type arg)
  {
    msg_.position_y = std::move(arg);
    return Init_IntersectionMsg_num_directions(msg_);
  }

private:
  ::vehicle_main::msg::IntersectionMsg msg_;
};

class Init_IntersectionMsg_position_x
{
public:
  Init_IntersectionMsg_position_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_IntersectionMsg_position_y position_x(::vehicle_main::msg::IntersectionMsg::_position_x_type arg)
  {
    msg_.position_x = std::move(arg);
    return Init_IntersectionMsg_position_y(msg_);
  }

private:
  ::vehicle_main::msg::IntersectionMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vehicle_main::msg::IntersectionMsg>()
{
  return vehicle_main::msg::builder::Init_IntersectionMsg_position_x();
}

}  // namespace vehicle_main

#endif  // VEHICLE_MAIN__MSG__DETAIL__INTERSECTION_MSG__BUILDER_HPP_
