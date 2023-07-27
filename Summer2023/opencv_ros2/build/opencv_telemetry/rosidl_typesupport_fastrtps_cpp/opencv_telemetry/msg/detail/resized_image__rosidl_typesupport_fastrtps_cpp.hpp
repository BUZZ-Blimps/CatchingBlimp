// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from opencv_telemetry:msg/ResizedImage.idl
// generated code does not contain a copyright notice

#ifndef OPENCV_TELEMETRY__MSG__DETAIL__RESIZED_IMAGE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define OPENCV_TELEMETRY__MSG__DETAIL__RESIZED_IMAGE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "opencv_telemetry/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "opencv_telemetry/msg/detail/resized_image__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace opencv_telemetry
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_opencv_telemetry
cdr_serialize(
  const opencv_telemetry::msg::ResizedImage & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_opencv_telemetry
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  opencv_telemetry::msg::ResizedImage & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_opencv_telemetry
get_serialized_size(
  const opencv_telemetry::msg::ResizedImage & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_opencv_telemetry
max_serialized_size_ResizedImage(
  bool & full_bounded,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace opencv_telemetry

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_opencv_telemetry
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, opencv_telemetry, msg, ResizedImage)();

#ifdef __cplusplus
}
#endif

#endif  // OPENCV_TELEMETRY__MSG__DETAIL__RESIZED_IMAGE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
