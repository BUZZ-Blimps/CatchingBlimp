// generated from rosidl_typesupport_connext_cpp/resource/idl__rosidl_typesupport_connext_cpp.hpp.em
// with input from yolo_msgs:msg/BoundingBoxes.idl
// generated code does not contain a copyright notice


#ifndef YOLO_MSGS__MSG__BOUNDING_BOXES__ROSIDL_TYPESUPPORT_CONNEXT_CPP_HPP_
#define YOLO_MSGS__MSG__BOUNDING_BOXES__ROSIDL_TYPESUPPORT_CONNEXT_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "yolo_msgs/msg/rosidl_typesupport_connext_cpp__visibility_control.h"
#include "yolo_msgs/msg/detail/bounding_boxes__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif

#include "yolo_msgs/msg/dds_connext/BoundingBoxes_Support.h"
#include "yolo_msgs/msg/dds_connext/BoundingBoxes_Plugin.h"
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
get_type_code__BoundingBoxes();

bool
ROSIDL_TYPESUPPORT_CONNEXT_CPP_PUBLIC_yolo_msgs
convert_ros_message_to_dds(
  const yolo_msgs::msg::BoundingBoxes & ros_message,
  yolo_msgs::msg::dds_::BoundingBoxes_ & dds_message);

bool
ROSIDL_TYPESUPPORT_CONNEXT_CPP_PUBLIC_yolo_msgs
convert_dds_message_to_ros(
  const yolo_msgs::msg::dds_::BoundingBoxes_ & dds_message,
  yolo_msgs::msg::BoundingBoxes & ros_message);

bool
to_cdr_stream__BoundingBoxes(
  const void * untyped_ros_message,
  ConnextStaticCDRStream * cdr_stream);

bool
to_message__BoundingBoxes(
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
  BoundingBoxes)();

#ifdef __cplusplus
}
#endif


#endif  // YOLO_MSGS__MSG__BOUNDING_BOXES__ROSIDL_TYPESUPPORT_CONNEXT_CPP_HPP_
