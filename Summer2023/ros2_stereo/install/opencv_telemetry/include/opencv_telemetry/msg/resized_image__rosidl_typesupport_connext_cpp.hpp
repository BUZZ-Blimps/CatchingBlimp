// generated from rosidl_typesupport_connext_cpp/resource/idl__rosidl_typesupport_connext_cpp.hpp.em
// with input from opencv_telemetry:msg/ResizedImage.idl
// generated code does not contain a copyright notice


#ifndef OPENCV_TELEMETRY__MSG__RESIZED_IMAGE__ROSIDL_TYPESUPPORT_CONNEXT_CPP_HPP_
#define OPENCV_TELEMETRY__MSG__RESIZED_IMAGE__ROSIDL_TYPESUPPORT_CONNEXT_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "opencv_telemetry/msg/rosidl_typesupport_connext_cpp__visibility_control.h"
#include "opencv_telemetry/msg/detail/resized_image__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif

#include "opencv_telemetry/msg/dds_connext/ResizedImage_Support.h"
#include "opencv_telemetry/msg/dds_connext/ResizedImage_Plugin.h"
#include "ndds/ndds_cpp.h"

#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// forward declaration of internal CDR Stream
struct ConnextStaticCDRStream;

// forward declaration of DDS types
class DDSDomainParticipant;
class DDSDataWriter;
class DDSDataReader;


namespace opencv_telemetry
{

namespace msg
{
namespace typesupport_connext_cpp
{

DDS_TypeCode *
get_type_code__ResizedImage();

bool
ROSIDL_TYPESUPPORT_CONNEXT_CPP_PUBLIC_opencv_telemetry
convert_ros_message_to_dds(
  const opencv_telemetry::msg::ResizedImage & ros_message,
  opencv_telemetry::msg::dds_::ResizedImage_ & dds_message);

bool
ROSIDL_TYPESUPPORT_CONNEXT_CPP_PUBLIC_opencv_telemetry
convert_dds_message_to_ros(
  const opencv_telemetry::msg::dds_::ResizedImage_ & dds_message,
  opencv_telemetry::msg::ResizedImage & ros_message);

bool
to_cdr_stream__ResizedImage(
  const void * untyped_ros_message,
  ConnextStaticCDRStream * cdr_stream);

bool
to_message__ResizedImage(
  const ConnextStaticCDRStream * cdr_stream,
  void * untyped_ros_message);

}  // namespace typesupport_connext_cpp

}  // namespace msg

}  // namespace opencv_telemetry


#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CONNEXT_CPP_PUBLIC_opencv_telemetry
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_connext_cpp,
  opencv_telemetry, msg,
  ResizedImage)();

#ifdef __cplusplus
}
#endif


#endif  // OPENCV_TELEMETRY__MSG__RESIZED_IMAGE__ROSIDL_TYPESUPPORT_CONNEXT_CPP_HPP_
