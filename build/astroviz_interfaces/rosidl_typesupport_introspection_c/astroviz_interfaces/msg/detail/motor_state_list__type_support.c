// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from astroviz_interfaces:msg/MotorStateList.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "astroviz_interfaces/msg/detail/motor_state_list__rosidl_typesupport_introspection_c.h"
#include "astroviz_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "astroviz_interfaces/msg/detail/motor_state_list__functions.h"
#include "astroviz_interfaces/msg/detail/motor_state_list__struct.h"


// Include directives for member types
// Member `motor_list`
#include "astroviz_interfaces/msg/motor_state.h"
// Member `motor_list`
#include "astroviz_interfaces/msg/detail/motor_state__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void astroviz_interfaces__msg__MotorStateList__rosidl_typesupport_introspection_c__MotorStateList_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  astroviz_interfaces__msg__MotorStateList__init(message_memory);
}

void astroviz_interfaces__msg__MotorStateList__rosidl_typesupport_introspection_c__MotorStateList_fini_function(void * message_memory)
{
  astroviz_interfaces__msg__MotorStateList__fini(message_memory);
}

size_t astroviz_interfaces__msg__MotorStateList__rosidl_typesupport_introspection_c__size_function__MotorStateList__motor_list(
  const void * untyped_member)
{
  const astroviz_interfaces__msg__MotorState__Sequence * member =
    (const astroviz_interfaces__msg__MotorState__Sequence *)(untyped_member);
  return member->size;
}

const void * astroviz_interfaces__msg__MotorStateList__rosidl_typesupport_introspection_c__get_const_function__MotorStateList__motor_list(
  const void * untyped_member, size_t index)
{
  const astroviz_interfaces__msg__MotorState__Sequence * member =
    (const astroviz_interfaces__msg__MotorState__Sequence *)(untyped_member);
  return &member->data[index];
}

void * astroviz_interfaces__msg__MotorStateList__rosidl_typesupport_introspection_c__get_function__MotorStateList__motor_list(
  void * untyped_member, size_t index)
{
  astroviz_interfaces__msg__MotorState__Sequence * member =
    (astroviz_interfaces__msg__MotorState__Sequence *)(untyped_member);
  return &member->data[index];
}

void astroviz_interfaces__msg__MotorStateList__rosidl_typesupport_introspection_c__fetch_function__MotorStateList__motor_list(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const astroviz_interfaces__msg__MotorState * item =
    ((const astroviz_interfaces__msg__MotorState *)
    astroviz_interfaces__msg__MotorStateList__rosidl_typesupport_introspection_c__get_const_function__MotorStateList__motor_list(untyped_member, index));
  astroviz_interfaces__msg__MotorState * value =
    (astroviz_interfaces__msg__MotorState *)(untyped_value);
  *value = *item;
}

void astroviz_interfaces__msg__MotorStateList__rosidl_typesupport_introspection_c__assign_function__MotorStateList__motor_list(
  void * untyped_member, size_t index, const void * untyped_value)
{
  astroviz_interfaces__msg__MotorState * item =
    ((astroviz_interfaces__msg__MotorState *)
    astroviz_interfaces__msg__MotorStateList__rosidl_typesupport_introspection_c__get_function__MotorStateList__motor_list(untyped_member, index));
  const astroviz_interfaces__msg__MotorState * value =
    (const astroviz_interfaces__msg__MotorState *)(untyped_value);
  *item = *value;
}

bool astroviz_interfaces__msg__MotorStateList__rosidl_typesupport_introspection_c__resize_function__MotorStateList__motor_list(
  void * untyped_member, size_t size)
{
  astroviz_interfaces__msg__MotorState__Sequence * member =
    (astroviz_interfaces__msg__MotorState__Sequence *)(untyped_member);
  astroviz_interfaces__msg__MotorState__Sequence__fini(member);
  return astroviz_interfaces__msg__MotorState__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember astroviz_interfaces__msg__MotorStateList__rosidl_typesupport_introspection_c__MotorStateList_message_member_array[1] = {
  {
    "motor_list",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(astroviz_interfaces__msg__MotorStateList, motor_list),  // bytes offset in struct
    NULL,  // default value
    astroviz_interfaces__msg__MotorStateList__rosidl_typesupport_introspection_c__size_function__MotorStateList__motor_list,  // size() function pointer
    astroviz_interfaces__msg__MotorStateList__rosidl_typesupport_introspection_c__get_const_function__MotorStateList__motor_list,  // get_const(index) function pointer
    astroviz_interfaces__msg__MotorStateList__rosidl_typesupport_introspection_c__get_function__MotorStateList__motor_list,  // get(index) function pointer
    astroviz_interfaces__msg__MotorStateList__rosidl_typesupport_introspection_c__fetch_function__MotorStateList__motor_list,  // fetch(index, &value) function pointer
    astroviz_interfaces__msg__MotorStateList__rosidl_typesupport_introspection_c__assign_function__MotorStateList__motor_list,  // assign(index, value) function pointer
    astroviz_interfaces__msg__MotorStateList__rosidl_typesupport_introspection_c__resize_function__MotorStateList__motor_list  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers astroviz_interfaces__msg__MotorStateList__rosidl_typesupport_introspection_c__MotorStateList_message_members = {
  "astroviz_interfaces__msg",  // message namespace
  "MotorStateList",  // message name
  1,  // number of fields
  sizeof(astroviz_interfaces__msg__MotorStateList),
  false,  // has_any_key_member_
  astroviz_interfaces__msg__MotorStateList__rosidl_typesupport_introspection_c__MotorStateList_message_member_array,  // message members
  astroviz_interfaces__msg__MotorStateList__rosidl_typesupport_introspection_c__MotorStateList_init_function,  // function to initialize message memory (memory has to be allocated)
  astroviz_interfaces__msg__MotorStateList__rosidl_typesupport_introspection_c__MotorStateList_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t astroviz_interfaces__msg__MotorStateList__rosidl_typesupport_introspection_c__MotorStateList_message_type_support_handle = {
  0,
  &astroviz_interfaces__msg__MotorStateList__rosidl_typesupport_introspection_c__MotorStateList_message_members,
  get_message_typesupport_handle_function,
  &astroviz_interfaces__msg__MotorStateList__get_type_hash,
  &astroviz_interfaces__msg__MotorStateList__get_type_description,
  &astroviz_interfaces__msg__MotorStateList__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_astroviz_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astroviz_interfaces, msg, MotorStateList)() {
  astroviz_interfaces__msg__MotorStateList__rosidl_typesupport_introspection_c__MotorStateList_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, astroviz_interfaces, msg, MotorState)();
  if (!astroviz_interfaces__msg__MotorStateList__rosidl_typesupport_introspection_c__MotorStateList_message_type_support_handle.typesupport_identifier) {
    astroviz_interfaces__msg__MotorStateList__rosidl_typesupport_introspection_c__MotorStateList_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &astroviz_interfaces__msg__MotorStateList__rosidl_typesupport_introspection_c__MotorStateList_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
