// generated from rosidl_typesupport_connext_cpp/resource/idl__dds_connext__type_support.cpp.em
// with input from opencv_telemetry:msg/ResizedImage.idl
// generated code does not contain a copyright notice

#include <limits>
#include <stdexcept>

#include "opencv_telemetry/msg/resized_image__rosidl_typesupport_connext_cpp.hpp"
#include "rcutils/types/uint8_array.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_connext_cpp/identifier.hpp"
#include "rosidl_typesupport_connext_cpp/message_type_support.h"
#include "rosidl_typesupport_connext_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_connext_cpp/wstring_conversion.hpp"

// forward declaration of message dependencies and their conversion functions
namespace sensor_msgs
{
namespace msg
{
namespace dds_
{
class Image_;
}  // namespace dds_

namespace typesupport_connext_cpp
{

bool convert_ros_message_to_dds(
  const sensor_msgs::msg::Image &,
  sensor_msgs::msg::dds_::Image_ &);
bool convert_dds_message_to_ros(
  const sensor_msgs::msg::dds_::Image_ &,
  sensor_msgs::msg::Image &);
}  // namespace typesupport_connext_cpp
}  // namespace msg
}  // namespace sensor_msgs


namespace opencv_telemetry
{

namespace msg
{

namespace typesupport_connext_cpp
{


DDS_TypeCode *
get_type_code__ResizedImage()
{
  return opencv_telemetry::msg::dds_::ResizedImage_TypeSupport::get_typecode();
}

bool
convert_ros_message_to_dds(
  const opencv_telemetry::msg::ResizedImage & ros_message,
  opencv_telemetry::msg::dds_::ResizedImage_ & dds_message)
{
  // member.name original_height
  dds_message.original_height_ =
    ros_message.original_height;

  // member.name original_width
  dds_message.original_width_ =
    ros_message.original_width;

  // member.name image
  if (
    !sensor_msgs::msg::typesupport_connext_cpp::convert_ros_message_to_dds(
      ros_message.image,
      dds_message.image_))
  {
    return false;
  }

  return true;
}

bool
convert_dds_message_to_ros(
  const opencv_telemetry::msg::dds_::ResizedImage_ & dds_message,
  opencv_telemetry::msg::ResizedImage & ros_message)
{
  // member.name original_height
  ros_message.original_height =
    dds_message.original_height_;

  // member.name original_width
  ros_message.original_width =
    dds_message.original_width_;

  // member.name image
  if (
    !sensor_msgs::msg::typesupport_connext_cpp::convert_dds_message_to_ros(
      dds_message.image_,
      ros_message.image))
  {
    return false;
  }

  return true;
}

bool
to_cdr_stream__ResizedImage(
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
  const opencv_telemetry::msg::ResizedImage & ros_message =
    *(const opencv_telemetry::msg::ResizedImage *)untyped_ros_message;

  // create a respective connext dds type
  opencv_telemetry::msg::dds_::ResizedImage_ * dds_message = opencv_telemetry::msg::dds_::ResizedImage_TypeSupport::create_data();
  if (!dds_message) {
    return false;
  }

  // convert ros to dds
  if (!convert_ros_message_to_dds(ros_message, *dds_message)) {
    return false;
  }

  // call the serialize function for the first time to get the expected length of the message
  unsigned int expected_length;
  if (opencv_telemetry::msg::dds_::ResizedImage_Plugin_serialize_to_cdr_buffer(
      NULL,
      &expected_length,
      dds_message) != RTI_TRUE)
  {
    fprintf(stderr, "failed to call opencv_telemetry::msg::dds_::ResizedImage_Plugin_serialize_to_cdr_buffer()\n");
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
  if (opencv_telemetry::msg::dds_::ResizedImage_Plugin_serialize_to_cdr_buffer(
      reinterpret_cast<char *>(cdr_stream->buffer),
      &buffer_length_uint,
      dds_message) != RTI_TRUE)
  {
    cdr_stream->buffer_length = 0;
    return false;
  }
  cdr_stream->buffer_length = expected_length;
  if (opencv_telemetry::msg::dds_::ResizedImage_TypeSupport::delete_data(dds_message) != DDS_RETCODE_OK) {
    return false;
  }
  return true;
}

bool
to_message__ResizedImage(
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

  opencv_telemetry::msg::dds_::ResizedImage_ * dds_message =
    opencv_telemetry::msg::dds_::ResizedImage_TypeSupport::create_data();
  if (cdr_stream->buffer_length > (std::numeric_limits<unsigned int>::max)()) {
    fprintf(stderr, "cdr_stream->buffer_length, unexpectedly larger than max unsigned int\n");
    return false;
  }
  if (opencv_telemetry::msg::dds_::ResizedImage_Plugin_deserialize_from_cdr_buffer(
      dds_message,
      reinterpret_cast<char *>(cdr_stream->buffer),
      static_cast<unsigned int>(cdr_stream->buffer_length)) != RTI_TRUE)
  {
    fprintf(stderr, "deserialize from cdr buffer failed\n");
    return false;
  }

  opencv_telemetry::msg::ResizedImage & ros_message =
    *(opencv_telemetry::msg::ResizedImage *)untyped_ros_message;
  bool success = convert_dds_message_to_ros(*dds_message, ros_message);
  if (opencv_telemetry::msg::dds_::ResizedImage_TypeSupport::delete_data(dds_message) != DDS_RETCODE_OK) {
    return false;
  }
  return success;
}

static message_type_support_callbacks_t _ResizedImage__callbacks = {
  "opencv_telemetry::msg",
  "ResizedImage",
  &get_type_code__ResizedImage,
  nullptr,
  nullptr,
  &to_cdr_stream__ResizedImage,
  &to_message__ResizedImage
};

static rosidl_message_type_support_t _ResizedImage__handle = {
  rosidl_typesupport_connext_cpp::typesupport_identifier,
  &_ResizedImage__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_connext_cpp

}  // namespace msg

}  // namespace opencv_telemetry


namespace rosidl_typesupport_connext_cpp
{

template<>
ROSIDL_TYPESUPPORT_CONNEXT_CPP_EXPORT_opencv_telemetry
const rosidl_message_type_support_t *
get_message_type_support_handle<opencv_telemetry::msg::ResizedImage>()
{
  return &opencv_telemetry::msg::typesupport_connext_cpp::_ResizedImage__handle;
}

}  // namespace rosidl_typesupport_connext_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_connext_cpp,
  opencv_telemetry, msg,
  ResizedImage)()
{
  return &opencv_telemetry::msg::typesupport_connext_cpp::_ResizedImage__handle;
}

#ifdef __cplusplus
}
#endif
