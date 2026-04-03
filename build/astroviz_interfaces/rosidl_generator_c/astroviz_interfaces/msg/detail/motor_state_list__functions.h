// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from astroviz_interfaces:msg/MotorStateList.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "astroviz_interfaces/msg/motor_state_list.h"


#ifndef ASTROVIZ_INTERFACES__MSG__DETAIL__MOTOR_STATE_LIST__FUNCTIONS_H_
#define ASTROVIZ_INTERFACES__MSG__DETAIL__MOTOR_STATE_LIST__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "astroviz_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "astroviz_interfaces/msg/detail/motor_state_list__struct.h"

/// Initialize msg/MotorStateList message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * astroviz_interfaces__msg__MotorStateList
 * )) before or use
 * astroviz_interfaces__msg__MotorStateList__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_astroviz_interfaces
bool
astroviz_interfaces__msg__MotorStateList__init(astroviz_interfaces__msg__MotorStateList * msg);

/// Finalize msg/MotorStateList message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astroviz_interfaces
void
astroviz_interfaces__msg__MotorStateList__fini(astroviz_interfaces__msg__MotorStateList * msg);

/// Create msg/MotorStateList message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * astroviz_interfaces__msg__MotorStateList__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_astroviz_interfaces
astroviz_interfaces__msg__MotorStateList *
astroviz_interfaces__msg__MotorStateList__create(void);

/// Destroy msg/MotorStateList message.
/**
 * It calls
 * astroviz_interfaces__msg__MotorStateList__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astroviz_interfaces
void
astroviz_interfaces__msg__MotorStateList__destroy(astroviz_interfaces__msg__MotorStateList * msg);

/// Check for msg/MotorStateList message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_astroviz_interfaces
bool
astroviz_interfaces__msg__MotorStateList__are_equal(const astroviz_interfaces__msg__MotorStateList * lhs, const astroviz_interfaces__msg__MotorStateList * rhs);

/// Copy a msg/MotorStateList message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_astroviz_interfaces
bool
astroviz_interfaces__msg__MotorStateList__copy(
  const astroviz_interfaces__msg__MotorStateList * input,
  astroviz_interfaces__msg__MotorStateList * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_astroviz_interfaces
const rosidl_type_hash_t *
astroviz_interfaces__msg__MotorStateList__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_astroviz_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
astroviz_interfaces__msg__MotorStateList__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_astroviz_interfaces
const rosidl_runtime_c__type_description__TypeSource *
astroviz_interfaces__msg__MotorStateList__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_astroviz_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
astroviz_interfaces__msg__MotorStateList__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/MotorStateList messages.
/**
 * It allocates the memory for the number of elements and calls
 * astroviz_interfaces__msg__MotorStateList__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_astroviz_interfaces
bool
astroviz_interfaces__msg__MotorStateList__Sequence__init(astroviz_interfaces__msg__MotorStateList__Sequence * array, size_t size);

/// Finalize array of msg/MotorStateList messages.
/**
 * It calls
 * astroviz_interfaces__msg__MotorStateList__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astroviz_interfaces
void
astroviz_interfaces__msg__MotorStateList__Sequence__fini(astroviz_interfaces__msg__MotorStateList__Sequence * array);

/// Create array of msg/MotorStateList messages.
/**
 * It allocates the memory for the array and calls
 * astroviz_interfaces__msg__MotorStateList__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_astroviz_interfaces
astroviz_interfaces__msg__MotorStateList__Sequence *
astroviz_interfaces__msg__MotorStateList__Sequence__create(size_t size);

/// Destroy array of msg/MotorStateList messages.
/**
 * It calls
 * astroviz_interfaces__msg__MotorStateList__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_astroviz_interfaces
void
astroviz_interfaces__msg__MotorStateList__Sequence__destroy(astroviz_interfaces__msg__MotorStateList__Sequence * array);

/// Check for msg/MotorStateList message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_astroviz_interfaces
bool
astroviz_interfaces__msg__MotorStateList__Sequence__are_equal(const astroviz_interfaces__msg__MotorStateList__Sequence * lhs, const astroviz_interfaces__msg__MotorStateList__Sequence * rhs);

/// Copy an array of msg/MotorStateList messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_astroviz_interfaces
bool
astroviz_interfaces__msg__MotorStateList__Sequence__copy(
  const astroviz_interfaces__msg__MotorStateList__Sequence * input,
  astroviz_interfaces__msg__MotorStateList__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ASTROVIZ_INTERFACES__MSG__DETAIL__MOTOR_STATE_LIST__FUNCTIONS_H_
