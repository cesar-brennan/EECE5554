// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from gps_msg:msg/GpsMsg.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "gps_msg/msg/detail/gps_msg__struct.h"
#include "gps_msg/msg/detail/gps_msg__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool gps_msg__msg__gps_msg__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[28];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("gps_msg.msg._gps_msg.GpsMsg", full_classname_dest, 27) == 0);
  }
  gps_msg__msg__GpsMsg * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // latitude
    PyObject * field = PyObject_GetAttrString(_pymsg, "latitude");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->latitude = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // longitude
    PyObject * field = PyObject_GetAttrString(_pymsg, "longitude");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->longitude = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // altitude
    PyObject * field = PyObject_GetAttrString(_pymsg, "altitude");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->altitude = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // hdop
    PyObject * field = PyObject_GetAttrString(_pymsg, "hdop");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->hdop = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // utm_easting
    PyObject * field = PyObject_GetAttrString(_pymsg, "utm_easting");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->utm_easting = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // utm_northing
    PyObject * field = PyObject_GetAttrString(_pymsg, "utm_northing");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->utm_northing = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // utc
    PyObject * field = PyObject_GetAttrString(_pymsg, "utc");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->utc = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // zone
    PyObject * field = PyObject_GetAttrString(_pymsg, "zone");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->zone = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // letter
    PyObject * field = PyObject_GetAttrString(_pymsg, "letter");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->letter, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * gps_msg__msg__gps_msg__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of GpsMsg */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("gps_msg.msg._gps_msg");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "GpsMsg");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  gps_msg__msg__GpsMsg * ros_message = (gps_msg__msg__GpsMsg *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // latitude
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->latitude);
    {
      int rc = PyObject_SetAttrString(_pymessage, "latitude", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // longitude
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->longitude);
    {
      int rc = PyObject_SetAttrString(_pymessage, "longitude", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // altitude
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->altitude);
    {
      int rc = PyObject_SetAttrString(_pymessage, "altitude", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // hdop
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->hdop);
    {
      int rc = PyObject_SetAttrString(_pymessage, "hdop", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // utm_easting
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->utm_easting);
    {
      int rc = PyObject_SetAttrString(_pymessage, "utm_easting", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // utm_northing
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->utm_northing);
    {
      int rc = PyObject_SetAttrString(_pymessage, "utm_northing", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // utc
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->utc);
    {
      int rc = PyObject_SetAttrString(_pymessage, "utc", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // zone
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->zone);
    {
      int rc = PyObject_SetAttrString(_pymessage, "zone", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // letter
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->letter.data,
      strlen(ros_message->letter.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "letter", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
