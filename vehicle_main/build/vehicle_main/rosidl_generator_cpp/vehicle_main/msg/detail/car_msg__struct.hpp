// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vehicle_main:msg/CarMsg.idl
// generated code does not contain a copyright notice

#ifndef VEHICLE_MAIN__MSG__DETAIL__CAR_MSG__STRUCT_HPP_
#define VEHICLE_MAIN__MSG__DETAIL__CAR_MSG__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__vehicle_main__msg__CarMsg __attribute__((deprecated))
#else
# define DEPRECATED__vehicle_main__msg__CarMsg __declspec(deprecated)
#endif

namespace vehicle_main
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CarMsg_
{
  using Type = CarMsg_<ContainerAllocator>;

  explicit CarMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->position_x = 0ul;
      this->position_y = 0ul;
      this->heading = 0ul;
      this->vehicle_speed = 0.0;
      this->priority = 0;
    }
  }

  explicit CarMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->position_x = 0ul;
      this->position_y = 0ul;
      this->heading = 0ul;
      this->vehicle_speed = 0.0;
      this->priority = 0;
    }
  }

  // field types and members
  using _position_x_type =
    uint32_t;
  _position_x_type position_x;
  using _position_y_type =
    uint32_t;
  _position_y_type position_y;
  using _heading_type =
    uint32_t;
  _heading_type heading;
  using _vehicle_speed_type =
    double;
  _vehicle_speed_type vehicle_speed;
  using _priority_type =
    uint8_t;
  _priority_type priority;

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
  Type & set__heading(
    const uint32_t & _arg)
  {
    this->heading = _arg;
    return *this;
  }
  Type & set__vehicle_speed(
    const double & _arg)
  {
    this->vehicle_speed = _arg;
    return *this;
  }
  Type & set__priority(
    const uint8_t & _arg)
  {
    this->priority = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vehicle_main::msg::CarMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const vehicle_main::msg::CarMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vehicle_main::msg::CarMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vehicle_main::msg::CarMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vehicle_main::msg::CarMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vehicle_main::msg::CarMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vehicle_main::msg::CarMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vehicle_main::msg::CarMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vehicle_main::msg::CarMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vehicle_main::msg::CarMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vehicle_main__msg__CarMsg
    std::shared_ptr<vehicle_main::msg::CarMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vehicle_main__msg__CarMsg
    std::shared_ptr<vehicle_main::msg::CarMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CarMsg_ & other) const
  {
    if (this->position_x != other.position_x) {
      return false;
    }
    if (this->position_y != other.position_y) {
      return false;
    }
    if (this->heading != other.heading) {
      return false;
    }
    if (this->vehicle_speed != other.vehicle_speed) {
      return false;
    }
    if (this->priority != other.priority) {
      return false;
    }
    return true;
  }
  bool operator!=(const CarMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CarMsg_

// alias to use template instance with default allocator
using CarMsg =
  vehicle_main::msg::CarMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vehicle_main

#endif  // VEHICLE_MAIN__MSG__DETAIL__CAR_MSG__STRUCT_HPP_
