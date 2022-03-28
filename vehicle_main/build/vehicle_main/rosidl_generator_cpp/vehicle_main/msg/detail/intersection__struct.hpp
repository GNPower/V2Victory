// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vehicle_main:msg/Intersection.idl
// generated code does not contain a copyright notice

#ifndef VEHICLE_MAIN__MSG__DETAIL__INTERSECTION__STRUCT_HPP_
#define VEHICLE_MAIN__MSG__DETAIL__INTERSECTION__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__vehicle_main__msg__Intersection __attribute__((deprecated))
#else
# define DEPRECATED__vehicle_main__msg__Intersection __declspec(deprecated)
#endif

namespace vehicle_main
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Intersection_
{
  using Type = Intersection_<ContainerAllocator>;

  explicit Intersection_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->position_x = 0ul;
      this->position_y = 0ul;
      this->num_directions = 0;
      std::fill<typename std::array<uint32_t, 32>::iterator, uint32_t>(this->directions.begin(), this->directions.end(), 0ul);
      this->intersection_state = 0;
      this->intersection_next_state = 0;
      this->intersection_switch_time = 0.0;
    }
  }

  explicit Intersection_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : directions(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->position_x = 0ul;
      this->position_y = 0ul;
      this->num_directions = 0;
      std::fill<typename std::array<uint32_t, 32>::iterator, uint32_t>(this->directions.begin(), this->directions.end(), 0ul);
      this->intersection_state = 0;
      this->intersection_next_state = 0;
      this->intersection_switch_time = 0.0;
    }
  }

  // field types and members
  using _position_x_type =
    uint32_t;
  _position_x_type position_x;
  using _position_y_type =
    uint32_t;
  _position_y_type position_y;
  using _num_directions_type =
    uint8_t;
  _num_directions_type num_directions;
  using _directions_type =
    std::array<uint32_t, 32>;
  _directions_type directions;
  using _intersection_state_type =
    uint8_t;
  _intersection_state_type intersection_state;
  using _intersection_next_state_type =
    uint8_t;
  _intersection_next_state_type intersection_next_state;
  using _intersection_switch_time_type =
    double;
  _intersection_switch_time_type intersection_switch_time;

  // setters for named parameter idiom
  Type & set__position_x(
    const uint32_t & _arg)
  {
    this->position_x = _arg;
    return *this;
  }
  Type & set__position_y(
    const uint32_t & _arg)
  {
    this->position_y = _arg;
    return *this;
  }
  Type & set__num_directions(
    const uint8_t & _arg)
  {
    this->num_directions = _arg;
    return *this;
  }
  Type & set__directions(
    const std::array<uint32_t, 32> & _arg)
  {
    this->directions = _arg;
    return *this;
  }
  Type & set__intersection_state(
    const uint8_t & _arg)
  {
    this->intersection_state = _arg;
    return *this;
  }
  Type & set__intersection_next_state(
    const uint8_t & _arg)
  {
    this->intersection_next_state = _arg;
    return *this;
  }
  Type & set__intersection_switch_time(
    const double & _arg)
  {
    this->intersection_switch_time = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vehicle_main::msg::Intersection_<ContainerAllocator> *;
  using ConstRawPtr =
    const vehicle_main::msg::Intersection_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vehicle_main::msg::Intersection_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vehicle_main::msg::Intersection_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vehicle_main::msg::Intersection_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vehicle_main::msg::Intersection_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vehicle_main::msg::Intersection_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vehicle_main::msg::Intersection_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vehicle_main::msg::Intersection_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vehicle_main::msg::Intersection_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vehicle_main__msg__Intersection
    std::shared_ptr<vehicle_main::msg::Intersection_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vehicle_main__msg__Intersection
    std::shared_ptr<vehicle_main::msg::Intersection_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Intersection_ & other) const
  {
    if (this->position_x != other.position_x) {
      return false;
    }
    if (this->position_y != other.position_y) {
      return false;
    }
    if (this->num_directions != other.num_directions) {
      return false;
    }
    if (this->directions != other.directions) {
      return false;
    }
    if (this->intersection_state != other.intersection_state) {
      return false;
    }
    if (this->intersection_next_state != other.intersection_next_state) {
      return false;
    }
    if (this->intersection_switch_time != other.intersection_switch_time) {
      return false;
    }
    return true;
  }
  bool operator!=(const Intersection_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Intersection_

// alias to use template instance with default allocator
using Intersection =
  vehicle_main::msg::Intersection_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vehicle_main

#endif  // VEHICLE_MAIN__MSG__DETAIL__INTERSECTION__STRUCT_HPP_
