// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vehicle_main:msg/Intersection.idl
// generated code does not contain a copyright notice

#ifndef VEHICLE_MAIN__MSG__DETAIL__INTERSECTION__BUILDER_HPP_
#define VEHICLE_MAIN__MSG__DETAIL__INTERSECTION__BUILDER_HPP_

#include "vehicle_main/msg/detail/intersection__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace vehicle_main
{

namespace msg
{

namespace builder
{

class Init_Intersection_intersection_switch_time
{
public:
  explicit Init_Intersection_intersection_switch_time(::vehicle_main::msg::Intersection & msg)
  : msg_(msg)
  {}
  ::vehicle_main::msg::Intersection intersection_switch_time(::vehicle_main::msg::Intersection::_intersection_switch_time_type arg)
  {
    msg_.intersection_switch_time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vehicle_main::msg::Intersection msg_;
};

class Init_Intersection_intersection_next_state
{
public:
  explicit Init_Intersection_intersection_next_state(::vehicle_main::msg::Intersection & msg)
  : msg_(msg)
  {}
  Init_Intersection_intersection_switch_time intersection_next_state(::vehicle_main::msg::Intersection::_intersection_next_state_type arg)
  {
    msg_.intersection_next_state = std::move(arg);
    return Init_Intersection_intersection_switch_time(msg_);
  }

private:
  ::vehicle_main::msg::Intersection msg_;
};

class Init_Intersection_intersection_state
{
public:
  explicit Init_Intersection_intersection_state(::vehicle_main::msg::Intersection & msg)
  : msg_(msg)
  {}
  Init_Intersection_intersection_next_state intersection_state(::vehicle_main::msg::Intersection::_intersection_state_type arg)
  {
    msg_.intersection_state = std::move(arg);
    return Init_Intersection_intersection_next_state(msg_);
  }

private:
  ::vehicle_main::msg::Intersection msg_;
};

class Init_Intersection_directions
{
public:
  explicit Init_Intersection_directions(::vehicle_main::msg::Intersection & msg)
  : msg_(msg)
  {}
  Init_Intersection_intersection_state directions(::vehicle_main::msg::Intersection::_directions_type arg)
  {
    msg_.directions = std::move(arg);
    return Init_Intersection_intersection_state(msg_);
  }

private:
  ::vehicle_main::msg::Intersection msg_;
};

class Init_Intersection_num_directions
{
public:
  explicit Init_Intersection_num_directions(::vehicle_main::msg::Intersection & msg)
  : msg_(msg)
  {}
  Init_Intersection_directions num_directions(::vehicle_main::msg::Intersection::_num_directions_type arg)
  {
    msg_.num_directions = std::move(arg);
    return Init_Intersection_directions(msg_);
  }

private:
  ::vehicle_main::msg::Intersection msg_;
};

class Init_Intersection_position_y
{
public:
  explicit Init_Intersection_position_y(::vehicle_main::msg::Intersection & msg)
  : msg_(msg)
  {}
  Init_Intersection_num_directions position_y(::vehicle_main::msg::Intersection::_position_y_type arg)
  {
    msg_.position_y = std::move(arg);
    return Init_Intersection_num_directions(msg_);
  }

private:
  ::vehicle_main::msg::Intersection msg_;
};

class Init_Intersection_position_x
{
public:
  Init_Intersection_position_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Intersection_position_y position_x(::vehicle_main::msg::Intersection::_position_x_type arg)
  {
    msg_.position_x = std::move(arg);
    return Init_Intersection_position_y(msg_);
  }

private:
  ::vehicle_main::msg::Intersection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vehicle_main::msg::Intersection>()
{
  return vehicle_main::msg::builder::Init_Intersection_position_x();
}

}  // namespace vehicle_main

#endif  // VEHICLE_MAIN__MSG__DETAIL__INTERSECTION__BUILDER_HPP_
