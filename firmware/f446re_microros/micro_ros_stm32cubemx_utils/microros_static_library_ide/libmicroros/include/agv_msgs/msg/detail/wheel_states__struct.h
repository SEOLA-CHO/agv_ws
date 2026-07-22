// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from agv_msgs:msg/WheelStates.idl
// generated code does not contain a copyright notice

#ifndef AGV_MSGS__MSG__DETAIL__WHEEL_STATES__STRUCT_H_
#define AGV_MSGS__MSG__DETAIL__WHEEL_STATES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/WheelStates in the package agv_msgs.
typedef struct agv_msgs__msg__WheelStates
{
  std_msgs__msg__Header header;
  float velocity_rad_s[4];
  int32_t erpm[4];
  int64_t tachometer_counts[4];
  bool online[4];
} agv_msgs__msg__WheelStates;

// Struct for a sequence of agv_msgs__msg__WheelStates.
typedef struct agv_msgs__msg__WheelStates__Sequence
{
  agv_msgs__msg__WheelStates * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} agv_msgs__msg__WheelStates__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AGV_MSGS__MSG__DETAIL__WHEEL_STATES__STRUCT_H_
