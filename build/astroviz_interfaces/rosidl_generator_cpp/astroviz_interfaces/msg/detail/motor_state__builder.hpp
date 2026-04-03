// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from astroviz_interfaces:msg/MotorState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "astroviz_interfaces/msg/motor_state.hpp"


#ifndef ASTROVIZ_INTERFACES__MSG__DETAIL__MOTOR_STATE__BUILDER_HPP_
#define ASTROVIZ_INTERFACES__MSG__DETAIL__MOTOR_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "astroviz_interfaces/msg/detail/motor_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace astroviz_interfaces
{

namespace msg
{

namespace builder
{

class Init_MotorState_velocity
{
public:
  explicit Init_MotorState_velocity(::astroviz_interfaces::msg::MotorState & msg)
  : msg_(msg)
  {}
  ::astroviz_interfaces::msg::MotorState velocity(::astroviz_interfaces::msg::MotorState::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::astroviz_interfaces::msg::MotorState msg_;
};

class Init_MotorState_position
{
public:
  explicit Init_MotorState_position(::astroviz_interfaces::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_velocity position(::astroviz_interfaces::msg::MotorState::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_MotorState_velocity(msg_);
  }

private:
  ::astroviz_interfaces::msg::MotorState msg_;
};

class Init_MotorState_voltage
{
public:
  explicit Init_MotorState_voltage(::astroviz_interfaces::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_position voltage(::astroviz_interfaces::msg::MotorState::_voltage_type arg)
  {
    msg_.voltage = std::move(arg);
    return Init_MotorState_position(msg_);
  }

private:
  ::astroviz_interfaces::msg::MotorState msg_;
};

class Init_MotorState_temperature
{
public:
  explicit Init_MotorState_temperature(::astroviz_interfaces::msg::MotorState & msg)
  : msg_(msg)
  {}
  Init_MotorState_voltage temperature(::astroviz_interfaces::msg::MotorState::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_MotorState_voltage(msg_);
  }

private:
  ::astroviz_interfaces::msg::MotorState msg_;
};

class Init_MotorState_name
{
public:
  Init_MotorState_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorState_temperature name(::astroviz_interfaces::msg::MotorState::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_MotorState_temperature(msg_);
  }

private:
  ::astroviz_interfaces::msg::MotorState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::astroviz_interfaces::msg::MotorState>()
{
  return astroviz_interfaces::msg::builder::Init_MotorState_name();
}

}  // namespace astroviz_interfaces

#endif  // ASTROVIZ_INTERFACES__MSG__DETAIL__MOTOR_STATE__BUILDER_HPP_
