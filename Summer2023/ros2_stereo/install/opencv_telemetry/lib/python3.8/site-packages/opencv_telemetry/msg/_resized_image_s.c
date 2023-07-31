// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from opencv_telemetry:msg/ResizedImage.idl
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
#include "opencv_telemetry/msg/detail/resized_image__struct.h"
#include "opencv_telemetry/msg/detail/resized_image__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool sensor_msgs__msg__image__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * sensor_msgs__msg__image__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool opencv_telemetry__msg__resized_image__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[49];
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
    assert(strncmp("opencv_telemetry.msg._resized_image.ResizedImage", full_classname_dest, 48) == 0);
  }
  opencv_telemetry__msg__ResizedImage * ros_message = _ros_message;
  {  // original_height
    PyObject * field = PyObject_GetAttrString(_pymsg, "original_height");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->original_height = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // original_width
    PyObject * field = PyObject_GetAttrString(_pymsg, "original_width");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->original_width = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // image
    PyObject * field = PyObject_GetAttrString(_pymsg, "image");
    if (!field) {
      return false;
    }
    if (!sensor_msgs__msg__image__convert_from_py(field, &ros_message->image)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * opencv_telemetry__msg__resized_image__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ResizedImage */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("opencv_telemetry.msg._resized_image");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ResizedImage");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  opencv_telemetry__msg__ResizedImage * ros_message = (opencv_telemetry__msg__ResizedImage *)raw_ros_message;
  {  // original_height
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->original_height);
    {
      int rc = PyObject_SetAttrString(_pymessage, "original_height", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // original_width
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->original_width);
    {
      int rc = PyObject_SetAttrString(_pymessage, "original_width", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // image
    PyObject * field = NULL;
    field = sensor_msgs__msg__image__convert_to_py(&ros_message->image);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "image", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
