// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vehicle_main:msg/Intersection.idl
// generated code does not contain a copyright notice

#ifndef VEHICLE_MAIN__MSG__DETAIL__INTERSECTION__TRAITS_HPP_
#define VEHICLE_MAIN__MSG__DETAIL__INTERSECTION__TRAITS_HPP_

#include "vehicle_main/msg/detail/intersection__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<vehicle_main::msg::Intersection>()
{
  return "vehicle_main::msg::Intersection";
}

template<>
inline const char * name<vehicle_main::msg::Intersection>()
{
  return "vehicle_main/msg/Intersection";
}

template<>
struct has_fixed_size<vehicle_main::msg::Intersection>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<vehicle_main::msg::Intersection>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<vehicle_main::msg::Intersection>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VEHICLE_MAIN__MSG__DETAIL__INTERSECTION__TRAITS_HPP_
