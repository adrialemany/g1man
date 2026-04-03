// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astroviz_interfaces:msg/MotorStateList.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "astroviz_interfaces/msg/motor_state_list.hpp"


#ifndef ASTROVIZ_INTERFACES__MSG__DETAIL__MOTOR_STATE_LIST__BUILDER_HPP_
#define ASTROVIZ_INTERFACES__MSG__DETAIL__MOTOR_STATE_LIST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "astroviz_interfaces/msg/detail/motor_state_list__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace astroviz_interfaces
{

namespace msg
{

namespace builder
{

class Init_MotorStateList_motor_list
{
public:
  Init_MotorStateList_motor_list()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::astroviz_interfaces::msg::MotorStateList motor_list(::astroviz_interfaces::msg::MotorStateList::_motor_list_type arg)
  {
    msg_.motor_list = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astroviz_interfaces::msg::MotorStateList msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::astroviz_interfaces::msg::MotorStateList>()
{
  return astroviz_interfaces::msg::builder::Init_MotorStateList_motor_list();
}

}  // namespace astroviz_interfaces

#endif  // ASTROVIZ_INTERFACES__MSG__DETAIL__MOTOR_STATE_LIST__BUILDER_HPP_
