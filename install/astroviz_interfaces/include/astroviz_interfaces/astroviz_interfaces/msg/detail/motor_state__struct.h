// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from astroviz_interfaces:msg/MotorState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "astroviz_interfaces/msg/motor_state.h"


#ifndef ASTROVIZ_INTERFACES__MSG__DETAIL__MOTOR_STATE__STRUCT_H_
#define ASTROVIZ_INTERFACES__MSG__DETAIL__MOTOR_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/MotorState in the package astroviz_interfaces.
typedef struct astroviz_interfaces__msg__MotorState
{
  rosidl_runtime_c__String name;
  float temperature;
  float voltage;
  float position;
  float velocity;
} astroviz_interfaces__msg__MotorState;

// Struct for a sequence of astroviz_interfaces__msg__MotorState.
typedef struct astroviz_interfaces__msg__MotorState__Sequence
{
  astroviz_interfaces__msg__MotorState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} astroviz_interfaces__msg__MotorState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASTROVIZ_INTERFACES__MSG__DETAIL__MOTOR_STATE__STRUCT_H_
