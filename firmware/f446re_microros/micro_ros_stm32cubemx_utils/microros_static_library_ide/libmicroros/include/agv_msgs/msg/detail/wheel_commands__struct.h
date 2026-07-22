// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from agv_msgs:msg/WheelCommands.idl
// generated code does not contain a copyright notice

#ifndef AGV_MSGS__MSG__DETAIL__WHEEL_COMMANDS__STRUCT_H_
#define AGV_MSGS__MSG__DETAIL__WHEEL_COMMANDS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/WheelCommands in the package agv_msgs.
typedef struct agv_msgs__msg__WheelCommands
{
  float velocity_rad_s[4];
} agv_msgs__msg__WheelCommands;

// Struct for a sequence of agv_msgs__msg__WheelCommands.
typedef struct agv_msgs__msg__WheelCommands__Sequence
{
  agv_msgs__msg__WheelCommands * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} agv_msgs__msg__WheelCommands__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AGV_MSGS__MSG__DETAIL__WHEEL_COMMANDS__STRUCT_H_
