// generated from rosidl_typesupport_connext_cpp/resource/idl__dds_connext__type_support.cpp.em
// with input from yolo_msgs:msg/BoundingBoxes.idl
// generated code does not contain a copyright notice

#include <limits>
#include <stdexcept>

#include "yolo_msgs/msg/bounding_boxes__rosidl_typesupport_connext_cpp.hpp"
#include "rcutils/types/uint8_array.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_connext_cpp/identifier.hpp"
#include "rosidl_typesupport_connext_cpp/message_type_support.h"
#include "rosidl_typesupport_connext_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_connext_cpp/wstring_conversion.hpp"

// forward declaration of message dependencies and their conversion functions
namespace yolo_msgs
{
namespace msg
{
namespace dds_
{
class BoundingBox_;
}  // namespace dds_

namespace typesupport_connext_cpp
{

bool convert_ros_message_to_dds(
  const yolo_msgs::msg::BoundingBox &,
  yolo_msgs::msg::dds_::BoundingBox_ &);
bool convert_dds_message_to_ros(
  const yolo_msgs::msg::dds_::BoundingBox_ &,
  yolo_msgs::msg::BoundingBox &);
}  // namespace typesupport_connext_cpp
}  // namespace msg
}  // namespace yolo_msgs


namespace yolo_msgs
{

namespace msg
{

namespace typesupport_connext_cpp
{


DDS_TypeCode *
get_type_code__BoundingBoxes()
{
  return yolo_msgs::msg::dds_::BoundingBoxes_TypeSupport::get_typecode();
}

bool
convert_ros_message_to_dds(
  const yolo_msgs::msg::BoundingBoxes & ros_message,
  yolo_msgs::msg::dds_::BoundingBoxes_ & dds_message)
{
  // member.name bounding_boxes
  {
    size_t size = ros_message.bounding_boxes.size();
    if (size > (std::numeric_limits<size_t>::max)()) {
      throw std::runtime_error("array size exceeds maximum DDS sequence size");
    }
    DDS_Long length = static_cast<DDS_Long>(size);
    if (length > dds_message.bounding_boxes_.maximum()) {
      if (!dds_message.bounding_boxes_.maximum(length)) {
        throw std::runtime_error("failed to set maximum of sequence");
      }
    }
    if (!dds_message.bounding_boxes_.length(length)) {
      throw std::runtime_error("failed to set length of sequence");
    }
    for (size_t i = 0; i < size; i++) {
      if (
        !yolo_msgs::msg::typesupport_connext_cpp::convert_ros_message_to_dds(
          ros_message.bounding_boxes[i],
          dds_message.bounding_boxes_[static_cast<DDS_Long>(i)]))
      {
        return false;
      }
    }
  }

  return true;
}

bool
convert_dds_message_to_ros(
  const yolo_msgs::msg::dds_::BoundingBoxes_ & dds_message,
  yolo_msgs::msg::BoundingBoxes & ros_message)
{
  // member.name bounding_boxes
  {
    size_t size = dds_message.bounding_boxes_.length();
    ros_message.bounding_boxes.resize(size);
    for (size_t i = 0; i < size; i++) {
      if (
        !yolo_msgs::msg::typesupport_connext_cpp::convert_dds_message_to_ros(
          dds_message.bounding_boxes_[static_cast<DDS_Long>(i)],
          ros_message.bounding_boxes[i]))
      {
        return false;
      }
    }
  }

  return true;
}

bool
to_cdr_stream__BoundingBoxes(
  const void * untyped_ros_message,
  rcutils_uint8_array_t * cdr_stream)
{
  if (!cdr_stream) {
    return false;
  }
  if (!untyped_ros_message) {
    return false;
  }

  // cast the untyped to the known ros message
  const yolo_msgs::msg::BoundingBoxes & ros_message =
    *(const yolo_msgs::msg::BoundingBoxes *)untyped_ros_message;

  // create a respective connext dds type
  yolo_msgs::msg::dds_::BoundingBoxes_ * dds_message = yolo_msgs::msg::dds_::BoundingBoxes_TypeSupport::create_data();
  if (!dds_message) {
    return false;
  }

  // convert ros to dds
  if (!convert_ros_message_to_dds(ros_message, *dds_message)) {
    return false;
  }

  // call the serialize function for the first time to get the expected length of the message
  unsigned int expected_length;
  if (yolo_msgs::msg::dds_::BoundingBoxes_Plugin_serialize_to_cdr_buffer(
      NULL,
      &expected_length,
      dds_message) != RTI_TRUE)
  {
    fprintf(stderr, "failed to call yolo_msgs::msg::dds_::BoundingBoxes_Plugin_serialize_to_cdr_buffer()\n");
    return false;
  }
  if (expected_length > (std::numeric_limits<unsigned int>::max)()) {
    fprintf(stderr, "expected_length, unexpectedly larger than max unsigned int\n");
    return false;
  }
  if (cdr_stream->buffer_capacity < expected_length) {
    uint8_t * new_buffer = static_cast<uint8_t *>(cdr_stream->allocator.allocate(expected_length, cdr_stream->allocator.state));
    if (NULL == new_buffer) {
      fprintf(stderr, "failed to allocate memory for cdr data\n");
      return false;
    }
    cdr_stream->allocator.deallocate(cdr_stream->buffer, cdr_stream->allocator.state);
    cdr_stream->buffer = new_buffer;
    cdr_stream->buffer_capacity = expected_length;
  }
  // call the function again and fill the buffer this time
  unsigned int buffer_length_uint = static_cast<unsigned int>(expected_length);
  if (yolo_msgs::msg::dds_::BoundingBoxes_Plugin_serialize_to_cdr_buffer(
      reinterpret_cast<char *>(cdr_stream->buffer),
      &buffer_length_uint,
      dds_message) != RTI_TRUE)
  {
    cdr_stream->buffer_length = 0;
    return false;
  }
  cdr_stream->buffer_length = expected_length;
  if (yolo_msgs::msg::dds_::BoundingBoxes_TypeSupport::delete_data(dds_message) != DDS_RETCODE_OK) {
    return false;
  }
  return true;
}

bool
to_message__BoundingBoxes(
  const rcutils_uint8_array_t * cdr_stream,
  void * untyped_ros_message)
{
  if (!cdr_stream) {
    return false;
  }
  if (!cdr_stream->buffer) {
    fprintf(stderr, "cdr stream doesn't contain data\n");
  }
  if (!untyped_ros_message) {
    return false;
  }

  yolo_msgs::msg::dds_::BoundingBoxes_ * dds_message =
    yolo_msgs::msg::dds_::BoundingBoxes_TypeSupport::create_data();
  if (cdr_stream->buffer_length > (std::numeric_limits<unsigned int>::max)()) {
    fprintf(stderr, "cdr_stream->buffer_length, unexpectedly larger than max unsigned int\n");
    return false;
  }
  if (yolo_msgs::msg::dds_::BoundingBoxes_Plugin_deserialize_from_cdr_buffer(
      dds_message,
      reinterpret_cast<char *>(cdr_stream->buffer),
      static_cast<unsigned int>(cdr_stream->buffer_length)) != RTI_TRUE)
  {
    fprintf(stderr, "deserialize from cdr buffer failed\n");
    return false;
  }

  yolo_msgs::msg::BoundingBoxes & ros_message =
    *(yolo_msgs::msg::BoundingBoxes *)untyped_ros_message;
  bool success = convert_dds_message_to_ros(*dds_message, ros_message);
  if (yolo_msgs::msg::dds_::BoundingBoxes_TypeSupport::delete_data(dds_message) != DDS_RETCODE_OK) {
    return false;
  }
  return success;
}

static message_type_support_callbacks_t _BoundingBoxes__callbacks = {
  "yolo_msgs::msg",
  "BoundingBoxes",
  &get_type_code__BoundingBoxes,
  nullptr,
  nullptr,
  &to_cdr_stream__BoundingBoxes,
  &to_message__BoundingBoxes
};

static rosidl_message_type_support_t _BoundingBoxes__handle = {
  rosidl_typesupport_connext_cpp::typesupport_identifier,
  &_BoundingBoxes__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_connext_cpp

}  // namespace msg

}  // namespace yolo_msgs


namespace rosidl_typesupport_connext_cpp
{

template<>
ROSIDL_TYPESUPPORT_CONNEXT_CPP_EXPORT_yolo_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<yolo_msgs::msg::BoundingBoxes>()
{
  return &yolo_msgs::msg::typesupport_connext_cpp::_BoundingBoxes__handle;
}

}  // namespace rosidl_typesupport_connext_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_connext_cpp,
  yolo_msgs, msg,
  BoundingBoxes)()
{
  return &yolo_msgs::msg::typesupport_connext_cpp::_BoundingBoxes__handle;
}

#ifdef __cplusplus
}
#endif
