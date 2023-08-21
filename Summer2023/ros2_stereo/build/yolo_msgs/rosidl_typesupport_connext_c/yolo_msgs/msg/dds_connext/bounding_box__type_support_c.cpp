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

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'header'
#include "std_msgs/msg/detail/header__functions.h"

// forward declare type support functions
// Member 'header'
ROSIDL_TYPESUPPORT_CONNEXT_C_IMPORT_yolo_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_connext_c,
  std_msgs, msg,
  Header)();

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
  // Member name: header
  {
    const message_type_support_callbacks_t * std_msgs__msg__Header__callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_connext_c, std_msgs, msg, Header
      )()->data);
    if (!std_msgs__msg__Header__callbacks->convert_ros_to_dds(
        &ros_message->header, &dds_message->header_))
    {
      return false;
    }
  }
  // Member name: x_center_balloon
  {
    dds_message->x_center_balloon_ = ros_message->x_center_balloon;
  }
  // Member name: y_center_balloon
  {
    dds_message->y_center_balloon_ = ros_message->y_center_balloon;
  }
  // Member name: width_balloon
  {
    dds_message->width_balloon_ = ros_message->width_balloon;
  }
  // Member name: height_balloon
  {
    dds_message->height_balloon_ = ros_message->height_balloon;
  }
  // Member name: x_center_y_goal
  {
    dds_message->x_center_y_goal_ = ros_message->x_center_y_goal;
  }
  // Member name: y_center_y_goal
  {
    dds_message->y_center_y_goal_ = ros_message->y_center_y_goal;
  }
  // Member name: width_y_goal
  {
    dds_message->width_y_goal_ = ros_message->width_y_goal;
  }
  // Member name: height_y_goal
  {
    dds_message->height_y_goal_ = ros_message->height_y_goal;
  }
  // Member name: x_center_o_goal
  {
    dds_message->x_center_o_goal_ = ros_message->x_center_o_goal;
  }
  // Member name: y_center_o_goal
  {
    dds_message->y_center_o_goal_ = ros_message->y_center_o_goal;
  }
  // Member name: width_o_goal
  {
    dds_message->width_o_goal_ = ros_message->width_o_goal;
  }
  // Member name: height_o_goal
  {
    dds_message->height_o_goal_ = ros_message->height_o_goal;
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
  // Member name: header
  {
    const rosidl_message_type_support_t * ts =
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
      rosidl_typesupport_connext_c,
      std_msgs, msg,
      Header)();
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(ts->data);
    callbacks->convert_dds_to_ros(&dds_message->header_, &ros_message->header);
  }
  // Member name: x_center_balloon
  {
    ros_message->x_center_balloon = dds_message->x_center_balloon_;
  }
  // Member name: y_center_balloon
  {
    ros_message->y_center_balloon = dds_message->y_center_balloon_;
  }
  // Member name: width_balloon
  {
    ros_message->width_balloon = dds_message->width_balloon_;
  }
  // Member name: height_balloon
  {
    ros_message->height_balloon = dds_message->height_balloon_;
  }
  // Member name: x_center_y_goal
  {
    ros_message->x_center_y_goal = dds_message->x_center_y_goal_;
  }
  // Member name: y_center_y_goal
  {
    ros_message->y_center_y_goal = dds_message->y_center_y_goal_;
  }
  // Member name: width_y_goal
  {
    ros_message->width_y_goal = dds_message->width_y_goal_;
  }
  // Member name: height_y_goal
  {
    ros_message->height_y_goal = dds_message->height_y_goal_;
  }
  // Member name: x_center_o_goal
  {
    ros_message->x_center_o_goal = dds_message->x_center_o_goal_;
  }
  // Member name: y_center_o_goal
  {
    ros_message->y_center_o_goal = dds_message->y_center_o_goal_;
  }
  // Member name: width_o_goal
  {
    ros_message->width_o_goal = dds_message->width_o_goal_;
  }
  // Member name: height_o_goal
  {
    ros_message->height_o_goal = dds_message->height_o_goal_;
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
