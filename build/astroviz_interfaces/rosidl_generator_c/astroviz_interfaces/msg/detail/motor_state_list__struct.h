// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from astroviz_interfaces:msg/MotorStateList.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "astroviz_interfaces/msg/motor_state_list.h"


#ifndef ASTROVIZ_INTERFACES__MSG__DETAIL__MOTOR_STATE_LIST__STRUCT_H_
#define ASTROVIZ_INTERFACES__MSG__DETAIL__MOTOR_STATE_LIST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'motor_list'
#include "astroviz_interfaces/msg/detail/motor_state__struct.h"

/// Struct defined in msg/MotorStateList in the package astroviz_interfaces.
typedef struct astroviz_interfaces__msg__MotorStateList
{
  astroviz_interfaces__msg__MotorState__Sequence motor_list;
} astroviz_interfaces__msg__MotorStateList;

// Struct for a sequence of astroviz_interfaces__msg__MotorStateList.
typedef struct astroviz_interfaces__msg__MotorStateList__Sequence
{
  astroviz_interfaces__msg__MotorStateList * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astroviz_interfaces__msg__MotorStateList__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASTROVIZ_INTERFACES__MSG__DETAIL__MOTOR_STATE_LIST__STRUCT_H_
