// generated from rosidl_typesupport_connext_cpp/resource/idl__rosidl_typesupport_connext_cpp.hpp.em
// with input from yolo_msgs:msg/BoundingBox.idl
// generated code does not contain a copyright notice


#ifndef YOLO_MSGS__MSG__BOUNDING_BOX__ROSIDL_TYPESUPPORT_CONNEXT_CPP_HPP_
#define YOLO_MSGS__MSG__BOUNDING_BOX__ROSIDL_TYPESUPPORT_CONNEXT_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "yolo_msgs/msg/rosidl_typesupport_connext_cpp__visibility_control.h"
#include "yolo_msgs/msg/detail/bounding_box__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif

#include "yolo_msgs/msg/dds_connext/BoundingBox_Support.h"
#include "yolo_msgs/msg/dds_connext/BoundingBox_Plugin.h"
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


namespace yolo_msgs
{

namespace msg
{
namespace typesupport_connext_cpp
{

DDS_TypeCode *
get_type_code__BoundingBox();

bool
ROSIDL_TYPESUPPORT_CONNEXT_CPP_PUBLIC_yolo_msgs
convert_ros_message_to_dds(
  const yolo_msgs::msg::BoundingBox & ros_message,
  yolo_msgs::msg::dds_::BoundingBox_ & dds_message);

bool
ROSIDL_TYPESUPPORT_CONNEXT_CPP_PUBLIC_yolo_msgs
convert_dds_message_to_ros(
  const yolo_msgs::msg::dds_::BoundingBox_ & dds_message,
  yolo_msgs::msg::BoundingBox & ros_message);

bool
to_cdr_stream__BoundingBox(
  const void * untyped_ros_message,
  ConnextStaticCDRStream * cdr_stream);

bool
to_message__BoundingBox(
  const ConnextStaticCDRStream * cdr_stream,
  void * untyped_ros_message);

}  // namespace typesupport_connext_cpp

}  // namespace msg

}  // namespace yolo_msgs


#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CONNEXT_CPP_PUBLIC_yolo_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_connext_cpp,
  yolo_msgs, msg,
  BoundingBox)();

#ifdef __cplusplus
}
#endif


#endif  // YOLO_MSGS__MSG__BOUNDING_BOX__ROSIDL_TYPESUPPORT_CONNEXT_CPP_HPP_
