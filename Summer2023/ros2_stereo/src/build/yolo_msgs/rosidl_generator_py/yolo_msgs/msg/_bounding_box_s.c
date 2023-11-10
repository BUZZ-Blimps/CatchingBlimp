// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from yolo_msgs:msg/BoundingBox.idl
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
#include "yolo_msgs/msg/detail/bounding_box__struct.h"
#include "yolo_msgs/msg/detail/bounding_box__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool yolo_msgs__msg__bounding_box__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[40];
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
    assert(strncmp("yolo_msgs.msg._bounding_box.BoundingBox", full_classname_dest, 39) == 0);
  }
  yolo_msgs__msg__BoundingBox * ros_message = _ros_message;
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
  {  // x_center_balloon
    PyObject * field = PyObject_GetAttrString(_pymsg, "x_center_balloon");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->x_center_balloon = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // y_center_balloon
    PyObject * field = PyObject_GetAttrString(_pymsg, "y_center_balloon");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->y_center_balloon = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // width_balloon
    PyObject * field = PyObject_GetAttrString(_pymsg, "width_balloon");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->width_balloon = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // height_balloon
    PyObject * field = PyObject_GetAttrString(_pymsg, "height_balloon");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->height_balloon = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // x_center_y_goal
    PyObject * field = PyObject_GetAttrString(_pymsg, "x_center_y_goal");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->x_center_y_goal = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // y_center_y_goal
    PyObject * field = PyObject_GetAttrString(_pymsg, "y_center_y_goal");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->y_center_y_goal = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // width_y_goal
    PyObject * field = PyObject_GetAttrString(_pymsg, "width_y_goal");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->width_y_goal = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // height_y_goal
    PyObject * field = PyObject_GetAttrString(_pymsg, "height_y_goal");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->height_y_goal = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // x_center_o_goal
    PyObject * field = PyObject_GetAttrString(_pymsg, "x_center_o_goal");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->x_center_o_goal = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // y_center_o_goal
    PyObject * field = PyObject_GetAttrString(_pymsg, "y_center_o_goal");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->y_center_o_goal = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // width_o_goal
    PyObject * field = PyObject_GetAttrString(_pymsg, "width_o_goal");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->width_o_goal = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }
  {  // height_o_goal
    PyObject * field = PyObject_GetAttrString(_pymsg, "height_o_goal");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->height_o_goal = PyLong_AsLongLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * yolo_msgs__msg__bounding_box__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of BoundingBox */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("yolo_msgs.msg._bounding_box");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "BoundingBox");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  yolo_msgs__msg__BoundingBox * ros_message = (yolo_msgs__msg__BoundingBox *)raw_ros_message;
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
  {  // x_center_balloon
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->x_center_balloon);
    {
      int rc = PyObject_SetAttrString(_pymessage, "x_center_balloon", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // y_center_balloon
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->y_center_balloon);
    {
      int rc = PyObject_SetAttrString(_pymessage, "y_center_balloon", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // width_balloon
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->width_balloon);
    {
      int rc = PyObject_SetAttrString(_pymessage, "width_balloon", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // height_balloon
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->height_balloon);
    {
      int rc = PyObject_SetAttrString(_pymessage, "height_balloon", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // x_center_y_goal
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->x_center_y_goal);
    {
      int rc = PyObject_SetAttrString(_pymessage, "x_center_y_goal", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // y_center_y_goal
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->y_center_y_goal);
    {
      int rc = PyObject_SetAttrString(_pymessage, "y_center_y_goal", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // width_y_goal
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->width_y_goal);
    {
      int rc = PyObject_SetAttrString(_pymessage, "width_y_goal", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // height_y_goal
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->height_y_goal);
    {
      int rc = PyObject_SetAttrString(_pymessage, "height_y_goal", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // x_center_o_goal
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->x_center_o_goal);
    {
      int rc = PyObject_SetAttrString(_pymessage, "x_center_o_goal", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // y_center_o_goal
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->y_center_o_goal);
    {
      int rc = PyObject_SetAttrString(_pymessage, "y_center_o_goal", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // width_o_goal
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->width_o_goal);
    {
      int rc = PyObject_SetAttrString(_pymessage, "width_o_goal", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // height_o_goal
    PyObject * field = NULL;
    field = PyLong_FromLongLong(ros_message->height_o_goal);
    {
      int rc = PyObject_SetAttrString(_pymessage, "height_o_goal", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
