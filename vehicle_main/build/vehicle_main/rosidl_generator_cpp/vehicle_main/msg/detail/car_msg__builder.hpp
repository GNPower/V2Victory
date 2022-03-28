// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vehicle_main:msg/CarMsg.idl
// generated code does not contain a copyright notice

#ifndef VEHICLE_MAIN__MSG__DETAIL__CAR_MSG__BUILDER_HPP_
#define VEHICLE_MAIN__MSG__DETAIL__CAR_MSG__BUILDER_HPP_

#include "vehicle_main/msg/detail/car_msg__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace vehicle_main
{

namespace msg
{

namespace builder
{

class Init_CarMsg_priority
{
public:
  explicit Init_CarMsg_priority(::vehicle_main::msg::CarMsg & msg)
  : msg_(msg)
  {}
  ::vehicle_main::msg::CarMsg priority(::vehicle_main::msg::CarMsg::_priority_type arg)
  {
    msg_.priority = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vehicle_main::msg::CarMsg msg_;
};

class Init_CarMsg_vehicle_speed
{
public:
  explicit Init_CarMsg_vehicle_speed(::vehicle_main::msg::CarMsg & msg)
  : msg_(msg)
  {}
  Init_CarMsg_priority vehicle_speed(::vehicle_main::msg::CarMsg::_vehicle_speed_type arg)
  {
    msg_.vehicle_speed = std::move(arg);
    return Init_CarMsg_priority(msg_);
  }

private:
  ::vehicle_main::msg::CarMsg msg_;
};

class Init_CarMsg_heading
{
public:
  explicit Init_CarMsg_heading(::vehicle_main::msg::CarMsg & msg)
  : msg_(msg)
  {}
  Init_CarMsg_vehicle_speed heading(::vehicle_main::msg::CarMsg::_heading_type arg)
  {
    msg_.heading = std::move(arg);
    return Init_CarMsg_vehicle_speed(msg_);
  }

private:
  ::vehicle_main::msg::CarMsg msg_;
};

class Init_CarMsg_position_y
{
public:
  explicit Init_CarMsg_position_y(::vehicle_main::msg::CarMsg & msg)
  : msg_(msg)
  {}
  Init_CarMsg_heading position_y(::vehicle_main::msg::CarMsg::_position_y_type arg)
  {
    msg_.position_y = std::move(arg);
    return Init_CarMsg_heading(msg_);
  }

private:
  ::vehicle_main::msg::CarMsg msg_;
};

class Init_CarMsg_position_x
{
public:
  Init_CarMsg_position_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CarMsg_position_y position_x(::vehicle_main::msg::CarMsg::_position_x_type arg)
  {
    msg_.position_x = std::move(arg);
    return Init_CarMsg_position_y(msg_);
  }

private:
  ::vehicle_main::msg::CarMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vehicle_main::msg::CarMsg>()
{
  return vehicle_main::msg::builder::Init_CarMsg_position_x();
}

}  // namespace vehicle_main

#endif  // VEHICLE_MAIN__MSG__DETAIL__CAR_MSG__BUILDER_HPP_
