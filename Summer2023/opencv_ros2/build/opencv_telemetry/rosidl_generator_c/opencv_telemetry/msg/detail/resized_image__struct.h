// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from opencv_telemetry:msg/ResizedImage.idl
// generated code does not contain a copyright notice

#ifndef OPENCV_TELEMETRY__MSG__DETAIL__RESIZED_IMAGE__STRUCT_H_
#define OPENCV_TELEMETRY__MSG__DETAIL__RESIZED_IMAGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'image'
#include "sensor_msgs/msg/detail/image__struct.h"

// Struct defined in msg/ResizedImage in the package opencv_telemetry.
typedef struct opencv_telemetry__msg__ResizedImage
{
  uint32_t original_height;
  uint32_t original_width;
  sensor_msgs__msg__Image image;
} opencv_telemetry__msg__ResizedImage;

// Struct for a sequence of opencv_telemetry__msg__ResizedImage.
typedef struct opencv_telemetry__msg__ResizedImage__Sequence
{
  opencv_telemetry__msg__ResizedImage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} opencv_telemetry__msg__ResizedImage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // OPENCV_TELEMETRY__MSG__DETAIL__RESIZED_IMAGE__STRUCT_H_
