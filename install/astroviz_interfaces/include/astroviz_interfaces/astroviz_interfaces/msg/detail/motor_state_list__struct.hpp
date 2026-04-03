// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from astroviz_interfaces:msg/MotorStateList.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "astroviz_interfaces/msg/motor_state_list.hpp"


#ifndef ASTROVIZ_INTERFACES__MSG__DETAIL__MOTOR_STATE_LIST__STRUCT_HPP_
#define ASTROVIZ_INTERFACES__MSG__DETAIL__MOTOR_STATE_LIST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'motor_list'
#include "astroviz_interfaces/msg/detail/motor_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__astroviz_interfaces__msg__MotorStateList __attribute__((deprecated))
#else
# define DEPRECATED__astroviz_interfaces__msg__MotorStateList __declspec(deprecated)
#endif

namespace astroviz_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorStateList_
{
  using Type = MotorStateList_<ContainerAllocator>;

  explicit MotorStateList_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit MotorStateList_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _motor_list_type =
    std::vector<astroviz_interfaces::msg::MotorState_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<astroviz_interfaces::msg::MotorState_<ContainerAllocator>>>;
  _motor_list_type motor_list;

  // setters for named parameter idiom
  Type & set__motor_list(
    const std::vector<astroviz_interfaces::msg::MotorState_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<astroviz_interfaces::msg::MotorState_<ContainerAllocator>>> & _arg)
  {
    this->motor_list = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    astroviz_interfaces::msg::MotorStateList_<ContainerAllocator> *;
  using ConstRawPtr =
    const astroviz_interfaces::msg::MotorStateList_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<astroviz_interfaces::msg::MotorStateList_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<astroviz_interfaces::msg::MotorStateList_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      astroviz_interfaces::msg::MotorStateList_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<astroviz_interfaces::msg::MotorStateList_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      astroviz_interfaces::msg::MotorStateList_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<astroviz_interfaces::msg::MotorStateList_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<astroviz_interfaces::msg::MotorStateList_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<astroviz_interfaces::msg::MotorStateList_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__astroviz_interfaces__msg__MotorStateList
    std::shared_ptr<astroviz_interfaces::msg::MotorStateList_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__astroviz_interfaces__msg__MotorStateList
    std::shared_ptr<astroviz_interfaces::msg::MotorStateList_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorStateList_ & other) const
  {
    if (this->motor_list != other.motor_list) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorStateList_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorStateList_

// alias to use template instance with default allocator
using MotorStateList =
  astroviz_interfaces::msg::MotorStateList_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace astroviz_interfaces

#endif  // ASTROVIZ_INTERFACES__MSG__DETAIL__MOTOR_STATE_LIST__STRUCT_HPP_
