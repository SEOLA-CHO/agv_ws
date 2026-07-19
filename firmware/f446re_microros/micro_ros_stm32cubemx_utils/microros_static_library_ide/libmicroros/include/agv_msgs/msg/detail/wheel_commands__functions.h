// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from agv_msgs:msg/WheelCommands.idl
// generated code does not contain a copyright notice

#ifndef AGV_MSGS__MSG__DETAIL__WHEEL_COMMANDS__FUNCTIONS_H_
#define AGV_MSGS__MSG__DETAIL__WHEEL_COMMANDS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "agv_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "agv_msgs/msg/detail/wheel_commands__struct.h"

/// Initialize msg/WheelCommands message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * agv_msgs__msg__WheelCommands
 * )) before or use
 * agv_msgs__msg__WheelCommands__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_agv_msgs
bool
agv_msgs__msg__WheelCommands__init(agv_msgs__msg__WheelCommands * msg);

/// Finalize msg/WheelCommands message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_agv_msgs
void
agv_msgs__msg__WheelCommands__fini(agv_msgs__msg__WheelCommands * msg);

/// Create msg/WheelCommands message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * agv_msgs__msg__WheelCommands__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_agv_msgs
agv_msgs__msg__WheelCommands *
agv_msgs__msg__WheelCommands__create();

/// Destroy msg/WheelCommands message.
/**
 * It calls
 * agv_msgs__msg__WheelCommands__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_agv_msgs
void
agv_msgs__msg__WheelCommands__destroy(agv_msgs__msg__WheelCommands * msg);

/// Check for msg/WheelCommands message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_agv_msgs
bool
agv_msgs__msg__WheelCommands__are_equal(const agv_msgs__msg__WheelCommands * lhs, const agv_msgs__msg__WheelCommands * rhs);

/// Copy a msg/WheelCommands message.
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
ROSIDL_GENERATOR_C_PUBLIC_agv_msgs
bool
agv_msgs__msg__WheelCommands__copy(
  const agv_msgs__msg__WheelCommands * input,
  agv_msgs__msg__WheelCommands * output);

/// Initialize array of msg/WheelCommands messages.
/**
 * It allocates the memory for the number of elements and calls
 * agv_msgs__msg__WheelCommands__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_agv_msgs
bool
agv_msgs__msg__WheelCommands__Sequence__init(agv_msgs__msg__WheelCommands__Sequence * array, size_t size);

/// Finalize array of msg/WheelCommands messages.
/**
 * It calls
 * agv_msgs__msg__WheelCommands__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_agv_msgs
void
agv_msgs__msg__WheelCommands__Sequence__fini(agv_msgs__msg__WheelCommands__Sequence * array);

/// Create array of msg/WheelCommands messages.
/**
 * It allocates the memory for the array and calls
 * agv_msgs__msg__WheelCommands__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_agv_msgs
agv_msgs__msg__WheelCommands__Sequence *
agv_msgs__msg__WheelCommands__Sequence__create(size_t size);

/// Destroy array of msg/WheelCommands messages.
/**
 * It calls
 * agv_msgs__msg__WheelCommands__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_agv_msgs
void
agv_msgs__msg__WheelCommands__Sequence__destroy(agv_msgs__msg__WheelCommands__Sequence * array);

/// Check for msg/WheelCommands message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_agv_msgs
bool
agv_msgs__msg__WheelCommands__Sequence__are_equal(const agv_msgs__msg__WheelCommands__Sequence * lhs, const agv_msgs__msg__WheelCommands__Sequence * rhs);

/// Copy an array of msg/WheelCommands messages.
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
ROSIDL_GENERATOR_C_PUBLIC_agv_msgs
bool
agv_msgs__msg__WheelCommands__Sequence__copy(
  const agv_msgs__msg__WheelCommands__Sequence * input,
  agv_msgs__msg__WheelCommands__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // AGV_MSGS__MSG__DETAIL__WHEEL_COMMANDS__FUNCTIONS_H_
