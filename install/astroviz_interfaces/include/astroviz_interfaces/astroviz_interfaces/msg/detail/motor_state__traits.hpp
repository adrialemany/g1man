// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from astroviz_interfaces:msg/MotorState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "astroviz_interfaces/msg/motor_state.hpp"


#ifndef ASTROVIZ_INTERFACES__MSG__DETAIL__MOTOR_STATE__TRAITS_HPP_
#define ASTROVIZ_INTERFACES__MSG__DETAIL__MOTOR_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "astroviz_interfaces/msg/detail/motor_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace astroviz_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotorState & msg,
  std::ostream & out)
{
  out << "{";
  // member: name
  {
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << ", ";
  }

  // member: temperature
  {
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
    out << ", ";
  }

  // member: voltage
  {
    out << "voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.voltage, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    rosidl_generator_traits::value_to_yaml(msg.position, out);
    out << ", ";
  }

  // member: velocity
  {
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotorState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << "\n";
  }

  // member: temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
    out << "\n";
  }

  // member: voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.voltage, out);
    out << "\n";
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position: ";
    rosidl_generator_traits::value_to_yaml(msg.position, out);
    out << "\n";
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotorState & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace astroviz_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use astroviz_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const astroviz_interfaces::msg::MotorState & msg,
  std::ostream & out, size_t indentation = 0)
{
  astroviz_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use astroviz_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const astroviz_interfaces::msg::MotorState & msg)
{
  return astroviz_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<astroviz_interfaces::msg::MotorState>()
{
  return "astroviz_interfaces::msg::MotorState";
}

template<>
inline const char * name<astroviz_interfaces::msg::MotorState>()
{
  return "astroviz_interfaces/msg/MotorState";
}

template<>
struct has_fixed_size<astroviz_interfaces::msg::MotorState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<astroviz_interfaces::msg::MotorState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<astroviz_interfaces::msg::MotorState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ASTROVIZ_INTERFACES__MSG__DETAIL__MOTOR_STATE__TRAITS_HPP_
