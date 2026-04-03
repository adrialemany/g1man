// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from astroviz_interfaces:msg/MotorStateList.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "astroviz_interfaces/msg/detail/motor_state_list__functions.h"
#include "astroviz_interfaces/msg/detail/motor_state_list__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace astroviz_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void MotorStateList_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) astroviz_interfaces::msg::MotorStateList(_init);
}

void MotorStateList_fini_function(void * message_memory)
{
  auto typed_message = static_cast<astroviz_interfaces::msg::MotorStateList *>(message_memory);
  typed_message->~MotorStateList();
}

size_t size_function__MotorStateList__motor_list(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<astroviz_interfaces::msg::MotorState> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MotorStateList__motor_list(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<astroviz_interfaces::msg::MotorState> *>(untyped_member);
  return &member[index];
}

void * get_function__MotorStateList__motor_list(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<astroviz_interfaces::msg::MotorState> *>(untyped_member);
  return &member[index];
}

void fetch_function__MotorStateList__motor_list(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const astroviz_interfaces::msg::MotorState *>(
    get_const_function__MotorStateList__motor_list(untyped_member, index));
  auto & value = *reinterpret_cast<astroviz_interfaces::msg::MotorState *>(untyped_value);
  value = item;
}

void assign_function__MotorStateList__motor_list(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<astroviz_interfaces::msg::MotorState *>(
    get_function__MotorStateList__motor_list(untyped_member, index));
  const auto & value = *reinterpret_cast<const astroviz_interfaces::msg::MotorState *>(untyped_value);
  item = value;
}

void resize_function__MotorStateList__motor_list(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<astroviz_interfaces::msg::MotorState> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MotorStateList_message_member_array[1] = {
  {
    "motor_list",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<astroviz_interfaces::msg::MotorState>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astroviz_interfaces::msg::MotorStateList, motor_list),  // bytes offset in struct
    nullptr,  // default value
    size_function__MotorStateList__motor_list,  // size() function pointer
    get_const_function__MotorStateList__motor_list,  // get_const(index) function pointer
    get_function__MotorStateList__motor_list,  // get(index) function pointer
    fetch_function__MotorStateList__motor_list,  // fetch(index, &value) function pointer
    assign_function__MotorStateList__motor_list,  // assign(index, value) function pointer
    resize_function__MotorStateList__motor_list  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MotorStateList_message_members = {
  "astroviz_interfaces::msg",  // message namespace
  "MotorStateList",  // message name
  1,  // number of fields
  sizeof(astroviz_interfaces::msg::MotorStateList),
  false,  // has_any_key_member_
  MotorStateList_message_member_array,  // message members
  MotorStateList_init_function,  // function to initialize message memory (memory has to be allocated)
  MotorStateList_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MotorStateList_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MotorStateList_message_members,
  get_message_typesupport_handle_function,
  &astroviz_interfaces__msg__MotorStateList__get_type_hash,
  &astroviz_interfaces__msg__MotorStateList__get_type_description,
  &astroviz_interfaces__msg__MotorStateList__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace astroviz_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<astroviz_interfaces::msg::MotorStateList>()
{
  return &::astroviz_interfaces::msg::rosidl_typesupport_introspection_cpp::MotorStateList_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, astroviz_interfaces, msg, MotorStateList)() {
  return &::astroviz_interfaces::msg::rosidl_typesupport_introspection_cpp::MotorStateList_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
