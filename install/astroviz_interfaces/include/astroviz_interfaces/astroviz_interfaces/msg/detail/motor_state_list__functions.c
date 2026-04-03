// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from astroviz_interfaces:msg/MotorStateList.idl
// generated code does not contain a copyright notice
#include "astroviz_interfaces/msg/detail/motor_state_list__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `motor_list`
#include "astroviz_interfaces/msg/detail/motor_state__functions.h"

bool
astroviz_interfaces__msg__MotorStateList__init(astroviz_interfaces__msg__MotorStateList * msg)
{
  if (!msg) {
    return false;
  }
  // motor_list
  if (!astroviz_interfaces__msg__MotorState__Sequence__init(&msg->motor_list, 0)) {
    astroviz_interfaces__msg__MotorStateList__fini(msg);
    return false;
  }
  return true;
}

void
astroviz_interfaces__msg__MotorStateList__fini(astroviz_interfaces__msg__MotorStateList * msg)
{
  if (!msg) {
    return;
  }
  // motor_list
  astroviz_interfaces__msg__MotorState__Sequence__fini(&msg->motor_list);
}

bool
astroviz_interfaces__msg__MotorStateList__are_equal(const astroviz_interfaces__msg__MotorStateList * lhs, const astroviz_interfaces__msg__MotorStateList * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // motor_list
  if (!astroviz_interfaces__msg__MotorState__Sequence__are_equal(
      &(lhs->motor_list), &(rhs->motor_list)))
  {
    return false;
  }
  return true;
}

bool
astroviz_interfaces__msg__MotorStateList__copy(
  const astroviz_interfaces__msg__MotorStateList * input,
  astroviz_interfaces__msg__MotorStateList * output)
{
  if (!input || !output) {
    return false;
  }
  // motor_list
  if (!astroviz_interfaces__msg__MotorState__Sequence__copy(
      &(input->motor_list), &(output->motor_list)))
  {
    return false;
  }
  return true;
}

astroviz_interfaces__msg__MotorStateList *
astroviz_interfaces__msg__MotorStateList__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astroviz_interfaces__msg__MotorStateList * msg = (astroviz_interfaces__msg__MotorStateList *)allocator.allocate(sizeof(astroviz_interfaces__msg__MotorStateList), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(astroviz_interfaces__msg__MotorStateList));
  bool success = astroviz_interfaces__msg__MotorStateList__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
astroviz_interfaces__msg__MotorStateList__destroy(astroviz_interfaces__msg__MotorStateList * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    astroviz_interfaces__msg__MotorStateList__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
astroviz_interfaces__msg__MotorStateList__Sequence__init(astroviz_interfaces__msg__MotorStateList__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astroviz_interfaces__msg__MotorStateList * data = NULL;

  if (size) {
    data = (astroviz_interfaces__msg__MotorStateList *)allocator.zero_allocate(size, sizeof(astroviz_interfaces__msg__MotorStateList), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = astroviz_interfaces__msg__MotorStateList__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        astroviz_interfaces__msg__MotorStateList__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
astroviz_interfaces__msg__MotorStateList__Sequence__fini(astroviz_interfaces__msg__MotorStateList__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      astroviz_interfaces__msg__MotorStateList__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

astroviz_interfaces__msg__MotorStateList__Sequence *
astroviz_interfaces__msg__MotorStateList__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  astroviz_interfaces__msg__MotorStateList__Sequence * array = (astroviz_interfaces__msg__MotorStateList__Sequence *)allocator.allocate(sizeof(astroviz_interfaces__msg__MotorStateList__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = astroviz_interfaces__msg__MotorStateList__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
astroviz_interfaces__msg__MotorStateList__Sequence__destroy(astroviz_interfaces__msg__MotorStateList__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    astroviz_interfaces__msg__MotorStateList__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
astroviz_interfaces__msg__MotorStateList__Sequence__are_equal(const astroviz_interfaces__msg__MotorStateList__Sequence * lhs, const astroviz_interfaces__msg__MotorStateList__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!astroviz_interfaces__msg__MotorStateList__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
astroviz_interfaces__msg__MotorStateList__Sequence__copy(
  const astroviz_interfaces__msg__MotorStateList__Sequence * input,
  astroviz_interfaces__msg__MotorStateList__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(astroviz_interfaces__msg__MotorStateList);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    astroviz_interfaces__msg__MotorStateList * data =
      (astroviz_interfaces__msg__MotorStateList *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!astroviz_interfaces__msg__MotorStateList__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          astroviz_interfaces__msg__MotorStateList__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!astroviz_interfaces__msg__MotorStateList__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
