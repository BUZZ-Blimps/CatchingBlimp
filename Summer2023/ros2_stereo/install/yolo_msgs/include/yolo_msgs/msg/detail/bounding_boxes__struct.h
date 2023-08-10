// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from yolo_msgs:msg/BoundingBoxes.idl
// generated code does not contain a copyright notice

#ifndef YOLO_MSGS__MSG__DETAIL__BOUNDING_BOXES__STRUCT_H_
#define YOLO_MSGS__MSG__DETAIL__BOUNDING_BOXES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'bounding_boxes'
#include "yolo_msgs/msg/detail/bounding_box__struct.h"

// Struct defined in msg/BoundingBoxes in the package yolo_msgs.
typedef struct yolo_msgs__msg__BoundingBoxes
{
  yolo_msgs__msg__BoundingBox__Sequence bounding_boxes;
} yolo_msgs__msg__BoundingBoxes;

// Struct for a sequence of yolo_msgs__msg__BoundingBoxes.
typedef struct yolo_msgs__msg__BoundingBoxes__Sequence
{
  yolo_msgs__msg__BoundingBoxes * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yolo_msgs__msg__BoundingBoxes__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // YOLO_MSGS__MSG__DETAIL__BOUNDING_BOXES__STRUCT_H_
