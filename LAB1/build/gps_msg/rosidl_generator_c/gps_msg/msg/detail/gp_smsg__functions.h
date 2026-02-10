// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from gps_msg:msg/GPSmsg.idl
// generated code does not contain a copyright notice

#ifndef GPS_MSG__MSG__DETAIL__GP_SMSG__FUNCTIONS_H_
#define GPS_MSG__MSG__DETAIL__GP_SMSG__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "gps_msg/msg/rosidl_generator_c__visibility_control.h"

#include "gps_msg/msg/detail/gp_smsg__struct.h"

/// Initialize msg/GPSmsg message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * gps_msg__msg__GPSmsg
 * )) before or use
 * gps_msg__msg__GPSmsg__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_gps_msg
bool
gps_msg__msg__GPSmsg__init(gps_msg__msg__GPSmsg * msg);

/// Finalize msg/GPSmsg message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gps_msg
void
gps_msg__msg__GPSmsg__fini(gps_msg__msg__GPSmsg * msg);

/// Create msg/GPSmsg message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * gps_msg__msg__GPSmsg__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_gps_msg
gps_msg__msg__GPSmsg *
gps_msg__msg__GPSmsg__create();

/// Destroy msg/GPSmsg message.
/**
 * It calls
 * gps_msg__msg__GPSmsg__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gps_msg
void
gps_msg__msg__GPSmsg__destroy(gps_msg__msg__GPSmsg * msg);

/// Check for msg/GPSmsg message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_gps_msg
bool
gps_msg__msg__GPSmsg__are_equal(const gps_msg__msg__GPSmsg * lhs, const gps_msg__msg__GPSmsg * rhs);

/// Copy a msg/GPSmsg message.
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
ROSIDL_GENERATOR_C_PUBLIC_gps_msg
bool
gps_msg__msg__GPSmsg__copy(
  const gps_msg__msg__GPSmsg * input,
  gps_msg__msg__GPSmsg * output);

/// Initialize array of msg/GPSmsg messages.
/**
 * It allocates the memory for the number of elements and calls
 * gps_msg__msg__GPSmsg__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_gps_msg
bool
gps_msg__msg__GPSmsg__Sequence__init(gps_msg__msg__GPSmsg__Sequence * array, size_t size);

/// Finalize array of msg/GPSmsg messages.
/**
 * It calls
 * gps_msg__msg__GPSmsg__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gps_msg
void
gps_msg__msg__GPSmsg__Sequence__fini(gps_msg__msg__GPSmsg__Sequence * array);

/// Create array of msg/GPSmsg messages.
/**
 * It allocates the memory for the array and calls
 * gps_msg__msg__GPSmsg__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_gps_msg
gps_msg__msg__GPSmsg__Sequence *
gps_msg__msg__GPSmsg__Sequence__create(size_t size);

/// Destroy array of msg/GPSmsg messages.
/**
 * It calls
 * gps_msg__msg__GPSmsg__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gps_msg
void
gps_msg__msg__GPSmsg__Sequence__destroy(gps_msg__msg__GPSmsg__Sequence * array);

/// Check for msg/GPSmsg message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_gps_msg
bool
gps_msg__msg__GPSmsg__Sequence__are_equal(const gps_msg__msg__GPSmsg__Sequence * lhs, const gps_msg__msg__GPSmsg__Sequence * rhs);

/// Copy an array of msg/GPSmsg messages.
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
ROSIDL_GENERATOR_C_PUBLIC_gps_msg
bool
gps_msg__msg__GPSmsg__Sequence__copy(
  const gps_msg__msg__GPSmsg__Sequence * input,
  gps_msg__msg__GPSmsg__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // GPS_MSG__MSG__DETAIL__GP_SMSG__FUNCTIONS_H_
