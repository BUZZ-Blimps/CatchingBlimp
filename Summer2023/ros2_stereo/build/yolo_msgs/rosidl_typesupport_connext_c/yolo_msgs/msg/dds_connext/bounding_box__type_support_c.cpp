// generated from rosidl_typesupport_connext_c/resource/idl__dds_connext__type_support_c.cpp.em
// with input from yolo_msgs:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#include <cassert>
#include <limits>

#include "yolo_msgs/msg/bounding_box__rosidl_typesupport_connext_c.h"
#include "rcutils/types/uint8_array.h"
#include "rosidl_typesupport_connext_c/identifier.h"
#include "rosidl_typesupport_connext_c/wstring_conversion.hpp"
#include "rosidl_typesupport_connext_cpp/message_type_support.h"
#include "yolo_msgs/msg/rosidl_typesupport_connext_c__visibility_control.h"
#include "yolo_msgs/msg/detail/bounding_box__struct.h"
#include "yolo_msgs/msg/detail/bounding_box__functions.h"

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

#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions
#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions

static DDS_TypeCode *
_BoundingBox__get_type_code()
{
  return yolo_msgs::msg::dds_::BoundingBox_TypeSupport::get_typecode();
}

static bool
_BoundingBox__convert_ros_to_dds(const void * untyped_ros_message, void * untyped_dds_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  if (!untyped_dds_message) {
    fprintf(stderr, "dds message handle is null\n");
    return false;
  }
  const yolo_msgs__msg__BoundingBox * ros_message =
    static_cast<const yolo_msgs__msg__BoundingBox *>(untyped_ros_message);
  yolo_msgs::msg::dds_::BoundingBox_ * dds_message =
    static_cast<yolo_msgs::msg::dds_::BoundingBox_ *>(untyped_dds_message);
  // Member name: probability
  {
    dds_message->probability_ = ros_message->probability;
  }
  // Member name: x_center
  {
    dds_message->x_center_ = ros_message->x_center;
  }
  // Member name: y_center
  {
    dds_message->y_center_ = ros_message->y_center;
  }
  // Member name: width
  {
    dds_message->width_ = ros_message->width;
  }
  // Member name: height
  {
    dds_message->height_ = ros_message->height;
  }
  // Member name: track_id
  {
    dds_message->track_id_ = ros_message->track_id;
  }
  // Member name: class_id
  {
    dds_message->class_id_ = ros_message->class_id;
  }
  return true;
}

static bool
_BoundingBox__convert_dds_to_ros(const void * untyped_dds_message, void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  if (!untyped_dds_message) {
    fprintf(stderr, "dds message handle is null\n");
    return false;
  }
  const yolo_msgs::msg::dds_::BoundingBox_ * dds_message =
    static_cast<const yolo_msgs::msg::dds_::BoundingBox_ *>(untyped_dds_message);
  yolo_msgs__msg__BoundingBox * ros_message =
    static_cast<yolo_msgs__msg__BoundingBox *>(untyped_ros_message);
  // Member name: probability
  {
    ros_message->probability = dds_message->probability_;
  }
  // Member name: x_center
  {
    ros_message->x_center = dds_message->x_center_;
  }
  // Member name: y_center
  {
    ros_message->y_center = dds_message->y_center_;
  }
  // Member name: width
  {
    ros_message->width = dds_message->width_;
  }
  // Member name: height
  {
    ros_message->height = dds_message->height_;
  }
  // Member name: track_id
  {
    ros_message->track_id = dds_message->track_id_;
  }
  // Member name: class_id
  {
    ros_message->class_id = dds_message->class_id_;
  }
  return true;
}


static bool
_BoundingBox__to_cdr_stream(
  const void * untyped_ros_message,
  rcutils_uint8_array_t * cdr_stream)
{
  if (!untyped_ros_message) {
    return false;
  }
  if (!cdr_stream) {
    return false;
  }
  const yolo_msgs__msg__BoundingBox * ros_message =
    static_cast<const yolo_msgs__msg__BoundingBox *>(untyped_ros_message);
  yolo_msgs::msg::dds_::BoundingBox_ dds_message;
  if (!_BoundingBox__convert_ros_to_dds(ros_message, &dds_message)) {
    return false;
  }

  // call the serialize function for the first time to get the expected length of the message
  unsigned int expected_length;
  if (yolo_msgs::msg::dds_::BoundingBox_Plugin_serialize_to_cdr_buffer(
      NULL, &expected_length, &dds_message) != RTI_TRUE)
  {
    fprintf(stderr, "failed to call yolo_msgs::msg::dds_::BoundingBox_Plugin_serialize_to_cdr_buffer()\n");
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
  if (yolo_msgs::msg::dds_::BoundingBox_Plugin_serialize_to_cdr_buffer(
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
_BoundingBox__to_message(
  const rcutils_uint8_array_t * cdr_stream,
  void * untyped_ros_message)
{
  if (!cdr_stream) {
    return false;
  }
  if (!untyped_ros_message) {
    return false;
  }

  yolo_msgs::msg::dds_::BoundingBox_ * dds_message =
    yolo_msgs::msg::dds_::BoundingBox_TypeSupport::create_data();
  if (cdr_stream->buffer_length > (std::numeric_limits<unsigned int>::max)()) {
    fprintf(stderr, "cdr_stream->buffer_length, unexpectedly larger than max unsigned int\n");
    return false;
  }
  if (yolo_msgs::msg::dds_::BoundingBox_Plugin_deserialize_from_cdr_buffer(
      dds_message,
      reinterpret_cast<char *>(cdr_stream->buffer),
      static_cast<unsigned int>(cdr_stream->buffer_length)) != RTI_TRUE)
  {
    fprintf(stderr, "deserialize from cdr buffer failed\n");
    return false;
  }
  bool success = _BoundingBox__convert_dds_to_ros(dds_message, untyped_ros_message);
  if (yolo_msgs::msg::dds_::BoundingBox_TypeSupport::delete_data(dds_message) != DDS_RETCODE_OK) {
    return false;
  }
  return success;
}

static message_type_support_callbacks_t _BoundingBox__callbacks = {
  "yolo_msgs::msg",  // message_namespace
  "BoundingBox",  // message_name
  _BoundingBox__get_type_code,  // get_type_code
  _BoundingBox__convert_ros_to_dds,  // convert_ros_to_dds
  _BoundingBox__convert_dds_to_ros,  // convert_dds_to_ros
  _BoundingBox__to_cdr_stream,  // to_cdr_stream
  _BoundingBox__to_message  // to_message
};

static rosidl_message_type_support_t _BoundingBox__type_support = {
  rosidl_typesupport_connext_c__identifier,
  &_BoundingBox__callbacks,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_connext_c,
  yolo_msgs, msg,
  BoundingBox)()
{
  return &_BoundingBox__type_support;
}

#if defined(__cplusplus)
}
#endif
