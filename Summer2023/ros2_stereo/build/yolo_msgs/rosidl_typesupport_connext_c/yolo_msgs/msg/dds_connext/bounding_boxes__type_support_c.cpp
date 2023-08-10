// generated from rosidl_typesupport_connext_c/resource/idl__dds_connext__type_support_c.cpp.em
// with input from yolo_msgs:msg/BoundingBoxes.idl
// generated code does not contain a copyright notice

#include <cassert>
#include <limits>

#include "yolo_msgs/msg/bounding_boxes__rosidl_typesupport_connext_c.h"
#include "rcutils/types/uint8_array.h"
#include "rosidl_typesupport_connext_c/identifier.h"
#include "rosidl_typesupport_connext_c/wstring_conversion.hpp"
#include "rosidl_typesupport_connext_cpp/message_type_support.h"
#include "yolo_msgs/msg/rosidl_typesupport_connext_c__visibility_control.h"
#include "yolo_msgs/msg/detail/bounding_boxes__struct.h"
#include "yolo_msgs/msg/detail/bounding_boxes__functions.h"

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

#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions
#if defined(__cplusplus)
extern "C"
{
#endif

// Include directives for member types
// Member 'bounding_boxes'
#include "yolo_msgs/msg/detail/bounding_box__struct.h"
// Member 'bounding_boxes'
#include "yolo_msgs/msg/detail/bounding_box__functions.h"

// forward declare type support functions
// Member 'bounding_boxes'
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_connext_c,
  yolo_msgs, msg,
  BoundingBox)();

static DDS_TypeCode *
_BoundingBoxes__get_type_code()
{
  return yolo_msgs::msg::dds_::BoundingBoxes_TypeSupport::get_typecode();
}

static bool
_BoundingBoxes__convert_ros_to_dds(const void * untyped_ros_message, void * untyped_dds_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  if (!untyped_dds_message) {
    fprintf(stderr, "dds message handle is null\n");
    return false;
  }
  const yolo_msgs__msg__BoundingBoxes * ros_message =
    static_cast<const yolo_msgs__msg__BoundingBoxes *>(untyped_ros_message);
  yolo_msgs::msg::dds_::BoundingBoxes_ * dds_message =
    static_cast<yolo_msgs::msg::dds_::BoundingBoxes_ *>(untyped_dds_message);
  // Member name: bounding_boxes
  {
    const message_type_support_callbacks_t * yolo_msgs__msg__BoundingBox__callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_connext_c, yolo_msgs, msg, BoundingBox
      )()->data);
    size_t size = ros_message->bounding_boxes.size;
    if (size > (std::numeric_limits<size_t>::max)()) {
      fprintf(stderr, "array size exceeds maximum DDS sequence size\n");
      return false;
    }
    DDS_Long length = static_cast<DDS_Long>(size);
    if (length > dds_message->bounding_boxes_.maximum()) {
      if (!dds_message->bounding_boxes_.maximum(length)) {
        fprintf(stderr, "failed to set maximum of sequence\n");
        return false;
      }
    }
    if (!dds_message->bounding_boxes_.length(length)) {
      fprintf(stderr, "failed to set length of sequence\n");
      return false;
    }
    for (DDS_Long i = 0; i < static_cast<DDS_Long>(size); ++i) {
      auto & ros_i = ros_message->bounding_boxes.data[i];
      if (!yolo_msgs__msg__BoundingBox__callbacks->convert_ros_to_dds(
          &ros_i, &dds_message->bounding_boxes_[i]))
      {
        return false;
      }
    }
  }
  return true;
}

static bool
_BoundingBoxes__convert_dds_to_ros(const void * untyped_dds_message, void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  if (!untyped_dds_message) {
    fprintf(stderr, "dds message handle is null\n");
    return false;
  }
  const yolo_msgs::msg::dds_::BoundingBoxes_ * dds_message =
    static_cast<const yolo_msgs::msg::dds_::BoundingBoxes_ *>(untyped_dds_message);
  yolo_msgs__msg__BoundingBoxes * ros_message =
    static_cast<yolo_msgs__msg__BoundingBoxes *>(untyped_ros_message);
  // Member name: bounding_boxes
  {
    DDS_Long size = dds_message->bounding_boxes_.length();
    if (ros_message->bounding_boxes.data) {
      yolo_msgs__msg__BoundingBox__Sequence__fini(&ros_message->bounding_boxes);
    }
    if (!yolo_msgs__msg__BoundingBox__Sequence__init(&ros_message->bounding_boxes, size)) {
      return "failed to create array for field 'bounding_boxes'";
    }
    for (DDS_Long i = 0; i < size; i++) {
      auto & ros_i = ros_message->bounding_boxes.data[i];
      const rosidl_message_type_support_t * ts =
        ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_connext_c,
        yolo_msgs, msg,
        BoundingBox)();
      const message_type_support_callbacks_t * callbacks =
        static_cast<const message_type_support_callbacks_t *>(ts->data);
      callbacks->convert_dds_to_ros(&dds_message->bounding_boxes_[i], &ros_i);
    }
  }
  return true;
}


static bool
_BoundingBoxes__to_cdr_stream(
  const void * untyped_ros_message,
  rcutils_uint8_array_t * cdr_stream)
{
  if (!untyped_ros_message) {
    return false;
  }
  if (!cdr_stream) {
    return false;
  }
  const yolo_msgs__msg__BoundingBoxes * ros_message =
    static_cast<const yolo_msgs__msg__BoundingBoxes *>(untyped_ros_message);
  yolo_msgs::msg::dds_::BoundingBoxes_ dds_message;
  if (!_BoundingBoxes__convert_ros_to_dds(ros_message, &dds_message)) {
    return false;
  }

  // call the serialize function for the first time to get the expected length of the message
  unsigned int expected_length;
  if (yolo_msgs::msg::dds_::BoundingBoxes_Plugin_serialize_to_cdr_buffer(
      NULL, &expected_length, &dds_message) != RTI_TRUE)
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
      &dds_message) != RTI_TRUE)
  {
    cdr_stream->buffer_length = 0;
    return false;
  }
  cdr_stream->buffer_length = expected_length;

  return true;
}

static bool
_BoundingBoxes__to_message(
  const rcutils_uint8_array_t * cdr_stream,
  void * untyped_ros_message)
{
  if (!cdr_stream) {
    return false;
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
  bool success = _BoundingBoxes__convert_dds_to_ros(dds_message, untyped_ros_message);
  if (yolo_msgs::msg::dds_::BoundingBoxes_TypeSupport::delete_data(dds_message) != DDS_RETCODE_OK) {
    return false;
  }
  return success;
}

static message_type_support_callbacks_t _BoundingBoxes__callbacks = {
  "yolo_msgs::msg",  // message_namespace
  "BoundingBoxes",  // message_name
  _BoundingBoxes__get_type_code,  // get_type_code
  _BoundingBoxes__convert_ros_to_dds,  // convert_ros_to_dds
  _BoundingBoxes__convert_dds_to_ros,  // convert_dds_to_ros
  _BoundingBoxes__to_cdr_stream,  // to_cdr_stream
  _BoundingBoxes__to_message  // to_message
};

static rosidl_message_type_support_t _BoundingBoxes__type_support = {
  rosidl_typesupport_connext_c__identifier,
  &_BoundingBoxes__callbacks,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_connext_c,
  yolo_msgs, msg,
  BoundingBoxes)()
{
  return &_BoundingBoxes__type_support;
}

#if defined(__cplusplus)
}
#endif
