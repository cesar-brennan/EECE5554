// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from gps_msg:msg/GPSmsg.idl
// generated code does not contain a copyright notice
#include "gps_msg/msg/detail/gp_smsg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `zone`
// Member `letter`
#include "rosidl_runtime_c/string_functions.h"

bool
gps_msg__msg__GPSmsg__init(gps_msg__msg__GPSmsg * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    gps_msg__msg__GPSmsg__fini(msg);
    return false;
  }
  // latitude
  // longitude
  // altitude
  // hdop
  // utm_easting
  // utm_northing
  // utc
  // zone
  if (!rosidl_runtime_c__String__init(&msg->zone)) {
    gps_msg__msg__GPSmsg__fini(msg);
    return false;
  }
  // letter
  if (!rosidl_runtime_c__String__init(&msg->letter)) {
    gps_msg__msg__GPSmsg__fini(msg);
    return false;
  }
  return true;
}

void
gps_msg__msg__GPSmsg__fini(gps_msg__msg__GPSmsg * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // latitude
  // longitude
  // altitude
  // hdop
  // utm_easting
  // utm_northing
  // utc
  // zone
  rosidl_runtime_c__String__fini(&msg->zone);
  // letter
  rosidl_runtime_c__String__fini(&msg->letter);
}

bool
gps_msg__msg__GPSmsg__are_equal(const gps_msg__msg__GPSmsg * lhs, const gps_msg__msg__GPSmsg * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // latitude
  if (lhs->latitude != rhs->latitude) {
    return false;
  }
  // longitude
  if (lhs->longitude != rhs->longitude) {
    return false;
  }
  // altitude
  if (lhs->altitude != rhs->altitude) {
    return false;
  }
  // hdop
  if (lhs->hdop != rhs->hdop) {
    return false;
  }
  // utm_easting
  if (lhs->utm_easting != rhs->utm_easting) {
    return false;
  }
  // utm_northing
  if (lhs->utm_northing != rhs->utm_northing) {
    return false;
  }
  // utc
  if (lhs->utc != rhs->utc) {
    return false;
  }
  // zone
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->zone), &(rhs->zone)))
  {
    return false;
  }
  // letter
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->letter), &(rhs->letter)))
  {
    return false;
  }
  return true;
}

bool
gps_msg__msg__GPSmsg__copy(
  const gps_msg__msg__GPSmsg * input,
  gps_msg__msg__GPSmsg * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // latitude
  output->latitude = input->latitude;
  // longitude
  output->longitude = input->longitude;
  // altitude
  output->altitude = input->altitude;
  // hdop
  output->hdop = input->hdop;
  // utm_easting
  output->utm_easting = input->utm_easting;
  // utm_northing
  output->utm_northing = input->utm_northing;
  // utc
  output->utc = input->utc;
  // zone
  if (!rosidl_runtime_c__String__copy(
      &(input->zone), &(output->zone)))
  {
    return false;
  }
  // letter
  if (!rosidl_runtime_c__String__copy(
      &(input->letter), &(output->letter)))
  {
    return false;
  }
  return true;
}

gps_msg__msg__GPSmsg *
gps_msg__msg__GPSmsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  gps_msg__msg__GPSmsg * msg = (gps_msg__msg__GPSmsg *)allocator.allocate(sizeof(gps_msg__msg__GPSmsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(gps_msg__msg__GPSmsg));
  bool success = gps_msg__msg__GPSmsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
gps_msg__msg__GPSmsg__destroy(gps_msg__msg__GPSmsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    gps_msg__msg__GPSmsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
gps_msg__msg__GPSmsg__Sequence__init(gps_msg__msg__GPSmsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  gps_msg__msg__GPSmsg * data = NULL;

  if (size) {
    data = (gps_msg__msg__GPSmsg *)allocator.zero_allocate(size, sizeof(gps_msg__msg__GPSmsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = gps_msg__msg__GPSmsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        gps_msg__msg__GPSmsg__fini(&data[i - 1]);
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
gps_msg__msg__GPSmsg__Sequence__fini(gps_msg__msg__GPSmsg__Sequence * array)
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
      gps_msg__msg__GPSmsg__fini(&array->data[i]);
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

gps_msg__msg__GPSmsg__Sequence *
gps_msg__msg__GPSmsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  gps_msg__msg__GPSmsg__Sequence * array = (gps_msg__msg__GPSmsg__Sequence *)allocator.allocate(sizeof(gps_msg__msg__GPSmsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = gps_msg__msg__GPSmsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
gps_msg__msg__GPSmsg__Sequence__destroy(gps_msg__msg__GPSmsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    gps_msg__msg__GPSmsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
gps_msg__msg__GPSmsg__Sequence__are_equal(const gps_msg__msg__GPSmsg__Sequence * lhs, const gps_msg__msg__GPSmsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!gps_msg__msg__GPSmsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
gps_msg__msg__GPSmsg__Sequence__copy(
  const gps_msg__msg__GPSmsg__Sequence * input,
  gps_msg__msg__GPSmsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(gps_msg__msg__GPSmsg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    gps_msg__msg__GPSmsg * data =
      (gps_msg__msg__GPSmsg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!gps_msg__msg__GPSmsg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          gps_msg__msg__GPSmsg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!gps_msg__msg__GPSmsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
