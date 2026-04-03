// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from astroviz_interfaces:msg/MotorStateList.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "astroviz_interfaces/msg/motor_state_list.hpp"


#ifndef ASTROVIZ_INTERFACES__MSG__DETAIL__MOTOR_STATE_LIST__TRAITS_HPP_
#define ASTROVIZ_INTERFACES__MSG__DETAIL__MOTOR_STATE_LIST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "astroviz_interfaces/msg/detail/motor_state_list__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'motor_list'
#include "astroviz_interfaces/msg/detail/motor_state__traits.hpp"

namespace astroviz_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotorStateList & msg,
  std::ostream & out)
{
  out << "{";
  // member: motor_list
  {
    if (msg.motor_list.size() == 0) {
      out << "motor_list: []";
    } else {
      out << "motor_list: [";
      size_t pending_items = msg.motor_list.size();
      for (auto item : msg.motor_list) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotorStateList & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: motor_list
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.motor_list.size() == 0) {
      out << "motor_list: []\n";
    } else {
      out << "motor_list:\n";
      for (auto item : msg.motor_list) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotorStateList & msg, bool use_flow_style = false)
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
  const astroviz_interfaces::msg::MotorStateList & msg,
  std::ostream & out, size_t indentation = 0)
{
  astroviz_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use astroviz_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const astroviz_interfaces::msg::MotorStateList & msg)
{
  return astroviz_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<astroviz_interfaces::msg::MotorStateList>()
{
  return "astroviz_interfaces::msg::MotorStateList";
}

template<>
inline const char * name<astroviz_interfaces::msg::MotorStateList>()
{
  return "astroviz_interfaces/msg/MotorStateList";
}

template<>
struct has_fixed_size<astroviz_interfaces::msg::MotorStateList>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<astroviz_interfaces::msg::MotorStateList>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<astroviz_interfaces::msg::MotorStateList>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ASTROVIZ_INTERFACES__MSG__DETAIL__MOTOR_STATE_LIST__TRAITS_HPP_
