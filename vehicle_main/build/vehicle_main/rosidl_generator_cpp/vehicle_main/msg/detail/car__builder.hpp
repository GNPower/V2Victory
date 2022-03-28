// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vehicle_main:msg/Car.idl
// generated code does not contain a copyright notice

#ifndef VEHICLE_MAIN__MSG__DETAIL__CAR__BUILDER_HPP_
#define VEHICLE_MAIN__MSG__DETAIL__CAR__BUILDER_HPP_

#include "vehicle_main/msg/detail/car__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace vehicle_main
{

namespace msg
{

namespace builder
{

class Init_Car_priority
{
public:
  explicit Init_Car_priority(::vehicle_main::msg::Car & msg)
  : msg_(msg)
  {}
  ::vehicle_main::msg::Car priority(::vehicle_main::msg::Car::_priority_type arg)
  {
    msg_.priority = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vehicle_main::msg::Car msg_;
};

class Init_Car_vehicle_speed
{
public:
  explicit Init_Car_vehicle_speed(::vehicle_main::msg::Car & msg)
  : msg_(msg)
  {}
  Init_Car_priority vehicle_speed(::vehicle_main::msg::Car::_vehicle_speed_type arg)
  {
    msg_.vehicle_speed = std::move(arg);
    return Init_Car_priority(msg_);
  }

private:
  ::vehicle_main::msg::Car msg_;
};

class Init_Car_heading
{
public:
  explicit Init_Car_heading(::vehicle_main::msg::Car & msg)
  : msg_(msg)
  {}
  Init_Car_vehicle_speed heading(::vehicle_main::msg::Car::_heading_type arg)
  {
    msg_.heading = std::move(arg);
    return Init_Car_vehicle_speed(msg_);
  }

private:
  ::vehicle_main::msg::Car msg_;
};

class Init_Car_position_y
{
public:
  explicit Init_Car_position_y(::vehicle_main::msg::Car & msg)
  : msg_(msg)
  {}
  Init_Car_heading position_y(::vehicle_main::msg::Car::_position_y_type arg)
  {
    msg_.position_y = std::move(arg);
    return Init_Car_heading(msg_);
  }

private:
  ::vehicle_main::msg::Car msg_;
};

class Init_Car_position_x
{
public:
  Init_Car_position_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Car_position_y position_x(::vehicle_main::msg::Car::_position_x_type arg)
  {
    msg_.position_x = std::move(arg);
    return Init_Car_position_y(msg_);
  }

private:
  ::vehicle_main::msg::Car msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vehicle_main::msg::Car>()
{
  return vehicle_main::msg::builder::Init_Car_position_x();
}

}  // namespace vehicle_main

#endif  // VEHICLE_MAIN__MSG__DETAIL__CAR__BUILDER_HPP_
