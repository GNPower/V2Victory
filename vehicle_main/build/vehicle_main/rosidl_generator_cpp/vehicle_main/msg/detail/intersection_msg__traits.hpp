// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vehicle_main:msg/IntersectionMsg.idl
// generated code does not contain a copyright notice

#ifndef VEHICLE_MAIN__MSG__DETAIL__INTERSECTION_MSG__TRAITS_HPP_
#define VEHICLE_MAIN__MSG__DETAIL__INTERSECTION_MSG__TRAITS_HPP_

#include "vehicle_main/msg/detail/intersection_msg__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<vehicle_main::msg::IntersectionMsg>()
{
  return "vehicle_main::msg::IntersectionMsg";
}

template<>
inline const char * name<vehicle_main::msg::IntersectionMsg>()
{
  return "vehicle_main/msg/IntersectionMsg";
}

template<>
struct has_fixed_size<vehicle_main::msg::IntersectionMsg>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<vehicle_main::msg::IntersectionMsg>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<vehicle_main::msg::IntersectionMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VEHICLE_MAIN__MSG__DETAIL__INTERSECTION_MSG__TRAITS_HPP_
