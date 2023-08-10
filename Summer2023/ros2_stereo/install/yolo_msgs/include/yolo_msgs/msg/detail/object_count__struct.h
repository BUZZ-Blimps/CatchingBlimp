// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from yolo_msgs:msg/ObjectCount.idl
// generated code does not contain a copyright notice

#ifndef YOLO_MSGS__MSG__DETAIL__OBJECT_COUNT__STRUCT_H_
#define YOLO_MSGS__MSG__DETAIL__OBJECT_COUNT__STRUCT_H_

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

// Struct defined in msg/ObjectCount in the package yolo_msgs.
typedef struct yolo_msgs__msg__ObjectCount
{
  std_msgs__msg__Header header;
  int8_t count;
} yolo_msgs__msg__ObjectCount;

// Struct for a sequence of yolo_msgs__msg__ObjectCount.
typedef struct yolo_msgs__msg__ObjectCount__Sequence
{
  yolo_msgs__msg__ObjectCount * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} yolo_msgs__msg__ObjectCount__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // YOLO_MSGS__MSG__DETAIL__OBJECT_COUNT__STRUCT_H_
