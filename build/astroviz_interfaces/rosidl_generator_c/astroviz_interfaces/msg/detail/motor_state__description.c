// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from astroviz_interfaces:msg/MotorState.idl
// generated code does not contain a copyright notice

#include "astroviz_interfaces/msg/detail/motor_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_astroviz_interfaces
const rosidl_type_hash_t *
astroviz_interfaces__msg__MotorState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x56, 0xde, 0x66, 0x1c, 0xbb, 0xc9, 0x31, 0x8a,
      0x36, 0xcf, 0x8f, 0x2d, 0x24, 0xa9, 0x83, 0x2e,
      0x1c, 0xc4, 0x9b, 0x69, 0x27, 0x0f, 0xcc, 0xea,
      0x22, 0xb4, 0xe6, 0x83, 0x84, 0xc8, 0xac, 0x51,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char astroviz_interfaces__msg__MotorState__TYPE_NAME[] = "astroviz_interfaces/msg/MotorState";

// Define type names, field names, and default values
static char astroviz_interfaces__msg__MotorState__FIELD_NAME__name[] = "name";
static char astroviz_interfaces__msg__MotorState__FIELD_NAME__temperature[] = "temperature";
static char astroviz_interfaces__msg__MotorState__FIELD_NAME__voltage[] = "voltage";
static char astroviz_interfaces__msg__MotorState__FIELD_NAME__position[] = "position";
static char astroviz_interfaces__msg__MotorState__FIELD_NAME__velocity[] = "velocity";

static rosidl_runtime_c__type_description__Field astroviz_interfaces__msg__MotorState__FIELDS[] = {
  {
    {astroviz_interfaces__msg__MotorState__FIELD_NAME__name, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {astroviz_interfaces__msg__MotorState__FIELD_NAME__temperature, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {astroviz_interfaces__msg__MotorState__FIELD_NAME__voltage, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {astroviz_interfaces__msg__MotorState__FIELD_NAME__position, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {astroviz_interfaces__msg__MotorState__FIELD_NAME__velocity, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
astroviz_interfaces__msg__MotorState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {astroviz_interfaces__msg__MotorState__TYPE_NAME, 34, 34},
      {astroviz_interfaces__msg__MotorState__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string name\n"
  "float32 temperature\n"
  "float32 voltage\n"
  "float32 position\n"
  "float32 velocity";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
astroviz_interfaces__msg__MotorState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {astroviz_interfaces__msg__MotorState__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 81, 81},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
astroviz_interfaces__msg__MotorState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *astroviz_interfaces__msg__MotorState__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
