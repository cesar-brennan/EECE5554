// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from gps_msg:msg/GPSmsg.idl
// generated code does not contain a copyright notice

#ifndef GPS_MSG__MSG__DETAIL__GP_SMSG__STRUCT_H_
#define GPS_MSG__MSG__DETAIL__GP_SMSG__STRUCT_H_

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
// Member 'zone'
// Member 'letter'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/GPSmsg in the package gps_msg.
typedef struct gps_msg__msg__GPSmsg
{
  std_msgs__msg__Header header;
  double latitude;
  double longitude;
  double altitude;
  double hdop;
  double utm_easting;
  double utm_northing;
  double utc;
  rosidl_runtime_c__String zone;
  rosidl_runtime_c__String letter;
} gps_msg__msg__GPSmsg;

// Struct for a sequence of gps_msg__msg__GPSmsg.
typedef struct gps_msg__msg__GPSmsg__Sequence
{
  gps_msg__msg__GPSmsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} gps_msg__msg__GPSmsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // GPS_MSG__MSG__DETAIL__GP_SMSG__STRUCT_H_
