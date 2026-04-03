// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from astroviz_interfaces:msg/MotorStateList.idl
// generated code does not contain a copyright notice

#include "astroviz_interfaces/msg/detail/motor_state_list__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_astroviz_interfaces
const rosidl_type_hash_t *
astroviz_interfaces__msg__MotorStateList__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x8e, 0x6e, 0x4b, 0x00, 0x25, 0xd6, 0x65, 0x3b,
      0xa5, 0x3f, 0x95, 0xef, 0xad, 0x8b, 0x31, 0xed,
      0x7c, 0x72, 0x9f, 0xdd, 0xfc, 0x7b, 0x1b, 0xa8,
      0xfb, 0x8a, 0xef, 0x3d, 0x45, 0xa6, 0xc6, 0x76,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "astroviz_interfaces/msg/detail/motor_state__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t astroviz_interfaces__msg__MotorState__EXPECTED_HASH = {1, {
    0x56, 0xde, 0x66, 0x1c, 0xbb, 0xc9, 0x31, 0x8a,
    0x36, 0xcf, 0x8f, 0x2d, 0x24, 0xa9, 0x83, 0x2e,
    0x1c, 0xc4, 0x9b, 0x69, 0x27, 0x0f, 0xcc, 0xea,
    0x22, 0xb4, 0xe6, 0x83, 0x84, 0xc8, 0xac, 0x51,
  }};
#endif

static char astroviz_interfaces__msg__MotorStateList__TYPE_NAME[] = "astroviz_interfaces/msg/MotorStateList";
static char astroviz_interfaces__msg__MotorState__TYPE_NAME[] = "astroviz_interfaces/msg/MotorState";

// Define type names, field names, and default values
static char astroviz_interfaces__msg__MotorStateList__FIELD_NAME__motor_list[] = "motor_list";

static rosidl_runtime_c__type_description__Field astroviz_interfaces__msg__MotorStateList__FIELDS[] = {
  {
    {astroviz_interfaces__msg__MotorStateList__FIELD_NAME__motor_list, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {astroviz_interfaces__msg__MotorState__TYPE_NAME, 34, 34},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription astroviz_interfaces__msg__MotorStateList__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {astroviz_interfaces__msg__MotorState__TYPE_NAME, 34, 34},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
astroviz_interfaces__msg__MotorStateList__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {astroviz_interfaces__msg__MotorStateList__TYPE_NAME, 38, 38},
      {astroviz_interfaces__msg__MotorStateList__FIELDS, 1, 1},
    },
    {astroviz_interfaces__msg__MotorStateList__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&astroviz_interfaces__msg__MotorState__EXPECTED_HASH, astroviz_interfaces__msg__MotorState__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = astroviz_interfaces__msg__MotorState__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "MotorState[] motor_list";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
astroviz_interfaces__msg__MotorStateList__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {astroviz_interfaces__msg__MotorStateList__TYPE_NAME, 38, 38},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 23, 23},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
astroviz_interfaces__msg__MotorStateList__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *astroviz_interfaces__msg__MotorStateList__get_individual_type_description_source(NULL),
    sources[1] = *astroviz_interfaces__msg__MotorState__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
