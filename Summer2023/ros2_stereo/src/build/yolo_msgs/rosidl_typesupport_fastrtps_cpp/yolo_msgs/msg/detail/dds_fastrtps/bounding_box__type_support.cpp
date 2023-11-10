// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from yolo_msgs:msg/BoundingBox.idl
// generated code does not contain a copyright notice
#include "yolo_msgs/msg/detail/bounding_box__rosidl_typesupport_fastrtps_cpp.hpp"
#include "yolo_msgs/msg/detail/bounding_box__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace std_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  std_msgs::msg::Header &);
size_t get_serialized_size(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_Header(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace std_msgs


namespace yolo_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_yolo_msgs
cdr_serialize(
  const yolo_msgs::msg::BoundingBox & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: x_center_balloon
  cdr << ros_message.x_center_balloon;
  // Member: y_center_balloon
  cdr << ros_message.y_center_balloon;
  // Member: width_balloon
  cdr << ros_message.width_balloon;
  // Member: height_balloon
  cdr << ros_message.height_balloon;
  // Member: x_center_y_goal
  cdr << ros_message.x_center_y_goal;
  // Member: y_center_y_goal
  cdr << ros_message.y_center_y_goal;
  // Member: width_y_goal
  cdr << ros_message.width_y_goal;
  // Member: height_y_goal
  cdr << ros_message.height_y_goal;
  // Member: x_center_o_goal
  cdr << ros_message.x_center_o_goal;
  // Member: y_center_o_goal
  cdr << ros_message.y_center_o_goal;
  // Member: width_o_goal
  cdr << ros_message.width_o_goal;
  // Member: height_o_goal
  cdr << ros_message.height_o_goal;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_yolo_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  yolo_msgs::msg::BoundingBox & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: x_center_balloon
  cdr >> ros_message.x_center_balloon;

  // Member: y_center_balloon
  cdr >> ros_message.y_center_balloon;

  // Member: width_balloon
  cdr >> ros_message.width_balloon;

  // Member: height_balloon
  cdr >> ros_message.height_balloon;

  // Member: x_center_y_goal
  cdr >> ros_message.x_center_y_goal;

  // Member: y_center_y_goal
  cdr >> ros_message.y_center_y_goal;

  // Member: width_y_goal
  cdr >> ros_message.width_y_goal;

  // Member: height_y_goal
  cdr >> ros_message.height_y_goal;

  // Member: x_center_o_goal
  cdr >> ros_message.x_center_o_goal;

  // Member: y_center_o_goal
  cdr >> ros_message.y_center_o_goal;

  // Member: width_o_goal
  cdr >> ros_message.width_o_goal;

  // Member: height_o_goal
  cdr >> ros_message.height_o_goal;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_yolo_msgs
get_serialized_size(
  const yolo_msgs::msg::BoundingBox & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header

  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);
  // Member: x_center_balloon
  {
    size_t item_size = sizeof(ros_message.x_center_balloon);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: y_center_balloon
  {
    size_t item_size = sizeof(ros_message.y_center_balloon);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: width_balloon
  {
    size_t item_size = sizeof(ros_message.width_balloon);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: height_balloon
  {
    size_t item_size = sizeof(ros_message.height_balloon);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: x_center_y_goal
  {
    size_t item_size = sizeof(ros_message.x_center_y_goal);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: y_center_y_goal
  {
    size_t item_size = sizeof(ros_message.y_center_y_goal);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: width_y_goal
  {
    size_t item_size = sizeof(ros_message.width_y_goal);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: height_y_goal
  {
    size_t item_size = sizeof(ros_message.height_y_goal);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: x_center_o_goal
  {
    size_t item_size = sizeof(ros_message.x_center_o_goal);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: y_center_o_goal
  {
    size_t item_size = sizeof(ros_message.y_center_o_goal);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: width_o_goal
  {
    size_t item_size = sizeof(ros_message.width_o_goal);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: height_o_goal
  {
    size_t item_size = sizeof(ros_message.height_o_goal);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_yolo_msgs
max_serialized_size_BoundingBox(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: header
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Header(
        full_bounded, current_alignment);
    }
  }

  // Member: x_center_balloon
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: y_center_balloon
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: width_balloon
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: height_balloon
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: x_center_y_goal
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: y_center_y_goal
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: width_y_goal
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: height_y_goal
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: x_center_o_goal
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: y_center_o_goal
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: width_o_goal
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: height_o_goal
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  return current_alignment - initial_alignment;
}

static bool _BoundingBox__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const yolo_msgs::msg::BoundingBox *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _BoundingBox__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<yolo_msgs::msg::BoundingBox *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _BoundingBox__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const yolo_msgs::msg::BoundingBox *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _BoundingBox__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_BoundingBox(full_bounded, 0);
}

static message_type_support_callbacks_t _BoundingBox__callbacks = {
  "yolo_msgs::msg",
  "BoundingBox",
  _BoundingBox__cdr_serialize,
  _BoundingBox__cdr_deserialize,
  _BoundingBox__get_serialized_size,
  _BoundingBox__max_serialized_size
};

static rosidl_message_type_support_t _BoundingBox__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_BoundingBox__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace yolo_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_yolo_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<yolo_msgs::msg::BoundingBox>()
{
  return &yolo_msgs::msg::typesupport_fastrtps_cpp::_BoundingBox__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, yolo_msgs, msg, BoundingBox)() {
  return &yolo_msgs::msg::typesupport_fastrtps_cpp::_BoundingBox__handle;
}

#ifdef __cplusplus
}
#endif
