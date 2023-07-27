// generated from rosidl_typesupport_connext_c/resource/idl__dds_connext__type_support_c.cpp.em
// with input from opencv_telemetry:msg/ResizedImage.idl
// generated code does not contain a copyright notice

#include <cassert>
#include <limits>

#include "opencv_telemetry/msg/resized_image__rosidl_typesupport_connext_c.h"
#include "rcutils/types/uint8_array.h"
#include "rosidl_typesupport_connext_c/identifier.h"
#include "rosidl_typesupport_connext_c/wstring_conversion.hpp"
#include "rosidl_typesupport_connext_cpp/message_type_support.h"
#include "opencv_telemetry/msg/rosidl_typesupport_connext_c__visibility_control.h"
#include "opencv_telemetry/msg/detail/resized_image__struct.h"
#include "opencv_telemetry/msg/detail/resized_image__functions.h"

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

#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions
#if defined(__cplusplus)
extern "C"
{
#endif

// Include directives for member types
// Member 'image'
#include "sensor_msgs/msg/detail/image__struct.h"
// Member 'image'
#include "sensor_msgs/msg/detail/image__functions.h"

// forward declare type support functions
// Member 'image'
ROSIDL_TYPESUPPORT_CONNEXT_C_IMPORT_opencv_telemetry
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_connext_c,
  sensor_msgs, msg,
  Image)();

static DDS_TypeCode *
_ResizedImage__get_type_code()
{
  return opencv_telemetry::msg::dds_::ResizedImage_TypeSupport::get_typecode();
}

static bool
_ResizedImage__convert_ros_to_dds(const void * untyped_ros_message, void * untyped_dds_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  if (!untyped_dds_message) {
    fprintf(stderr, "dds message handle is null\n");
    return false;
  }
  const opencv_telemetry__msg__ResizedImage * ros_message =
    static_cast<const opencv_telemetry__msg__ResizedImage *>(untyped_ros_message);
  opencv_telemetry::msg::dds_::ResizedImage_ * dds_message =
    static_cast<opencv_telemetry::msg::dds_::ResizedImage_ *>(untyped_dds_message);
  // Member name: original_height
  {
    dds_message->original_height_ = ros_message->original_height;
  }
  // Member name: original_width
  {
    dds_message->original_width_ = ros_message->original_width;
  }
  // Member name: image
  {
    const message_type_support_callbacks_t * sensor_msgs__msg__Image__callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_connext_c, sensor_msgs, msg, Image
      )()->data);
    if (!sensor_msgs__msg__Image__callbacks->convert_ros_to_dds(
        &ros_message->image, &dds_message->image_))
    {
      return false;
    }
  }
  return true;
}

static bool
_ResizedImage__convert_dds_to_ros(const void * untyped_dds_message, void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  if (!untyped_dds_message) {
    fprintf(stderr, "dds message handle is null\n");
    return false;
  }
  const opencv_telemetry::msg::dds_::ResizedImage_ * dds_message =
    static_cast<const opencv_telemetry::msg::dds_::ResizedImage_ *>(untyped_dds_message);
  opencv_telemetry__msg__ResizedImage * ros_message =
    static_cast<opencv_telemetry__msg__ResizedImage *>(untyped_ros_message);
  // Member name: original_height
  {
    ros_message->original_height = dds_message->original_height_;
  }
  // Member name: original_width
  {
    ros_message->original_width = dds_message->original_width_;
  }
  // Member name: image
  {
    const rosidl_message_type_support_t * ts =
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
      rosidl_typesupport_connext_c,
      sensor_msgs, msg,
      Image)();
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(ts->data);
    callbacks->convert_dds_to_ros(&dds_message->image_, &ros_message->image);
  }
  return true;
}


static bool
_ResizedImage__to_cdr_stream(
  const void * untyped_ros_message,
  rcutils_uint8_array_t * cdr_stream)
{
  if (!untyped_ros_message) {
    return false;
  }
  if (!cdr_stream) {
    return false;
  }
  const opencv_telemetry__msg__ResizedImage * ros_message =
    static_cast<const opencv_telemetry__msg__ResizedImage *>(untyped_ros_message);
  opencv_telemetry::msg::dds_::ResizedImage_ dds_message;
  if (!_ResizedImage__convert_ros_to_dds(ros_message, &dds_message)) {
    return false;
  }

  // call the serialize function for the first time to get the expected length of the message
  unsigned int expected_length;
  if (opencv_telemetry::msg::dds_::ResizedImage_Plugin_serialize_to_cdr_buffer(
      NULL, &expected_length, &dds_message) != RTI_TRUE)
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
      &dds_message) != RTI_TRUE)
  {
    cdr_stream->buffer_length = 0;
    return false;
  }
  cdr_stream->buffer_length = expected_length;

  return true;
}

static bool
_ResizedImage__to_message(
  const rcutils_uint8_array_t * cdr_stream,
  void * untyped_ros_message)
{
  if (!cdr_stream) {
    return false;
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
  bool success = _ResizedImage__convert_dds_to_ros(dds_message, untyped_ros_message);
  if (opencv_telemetry::msg::dds_::ResizedImage_TypeSupport::delete_data(dds_message) != DDS_RETCODE_OK) {
    return false;
  }
  return success;
}

static message_type_support_callbacks_t _ResizedImage__callbacks = {
  "opencv_telemetry::msg",  // message_namespace
  "ResizedImage",  // message_name
  _ResizedImage__get_type_code,  // get_type_code
  _ResizedImage__convert_ros_to_dds,  // convert_ros_to_dds
  _ResizedImage__convert_dds_to_ros,  // convert_dds_to_ros
  _ResizedImage__to_cdr_stream,  // to_cdr_stream
  _ResizedImage__to_message  // to_message
};

static rosidl_message_type_support_t _ResizedImage__type_support = {
  rosidl_typesupport_connext_c__identifier,
  &_ResizedImage__callbacks,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_connext_c,
  opencv_telemetry, msg,
  ResizedImage)()
{
  return &_ResizedImage__type_support;
}

#if defined(__cplusplus)
}
#endif
