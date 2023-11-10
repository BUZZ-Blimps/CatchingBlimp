// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from yolo_msgs:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef YOLO_MSGS__MSG__DETAIL__BOUNDING_BOX__STRUCT_H_
#define YOLO_MSGS__MSG__DETAIL__BOUNDING_BOX__STRUCT_H_

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

// Struct defined in msg/BoundingBox in the package yolo_msgs.
typedef struct yolo_msgs__msg__BoundingBox
{
  std_msgs__msg__Header header;
  int64_t x_center_balloon;
  int64_t y_center_balloon;
  int64_t width_balloon;
  int64_t height_balloon;
  int64_t x_center_y_goal;
  int64_t y_center_y_goal;
  int64_t width_y_goal;
  int64_t height_y_goal;
  int64_t x_center_o_goal;
  int64_t y_center_o_goal;
  int64_t width_o_goal;
  int64_t height_o_goal;
} yolo_msgs__msg__BoundingBox;

// Struct for a sequence of yolo_msgs__msg__BoundingBox.
typedef struct yolo_msgs__msg__BoundingBox__Sequence
{
  yolo_msgs__msg__BoundingBox * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yolo_msgs__msg__BoundingBox__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // YOLO_MSGS__MSG__DETAIL__BOUNDING_BOX__STRUCT_H_
