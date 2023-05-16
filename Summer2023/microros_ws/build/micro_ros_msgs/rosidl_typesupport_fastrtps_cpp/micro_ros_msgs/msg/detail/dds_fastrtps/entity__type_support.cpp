// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from micro_ros_msgs:msg/Entity.idl
// generated code does not contain a copyright notice
#include "micro_ros_msgs/msg/detail/entity__rosidl_typesupport_fastrtps_cpp.hpp"
#include "micro_ros_msgs/msg/detail/entity__struct.hpp"

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

namespace micro_ros_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_micro_ros_msgs
cdr_serialize(
  const micro_ros_msgs::msg::Entity & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: entity_type
  cdr << ros_message.entity_type;
  // Member: name
  cdr << ros_message.name;
  // Member: types
  {
    cdr << ros_message.types;
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_micro_ros_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  micro_ros_msgs::msg::Entity & ros_message)
{
  // Member: entity_type
  cdr >> ros_message.entity_type;

  // Member: name
  cdr >> ros_message.name;

  // Member: types
  {
    cdr >> ros_message.types;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_micro_ros_msgs
get_serialized_size(
  const micro_ros_msgs::msg::Entity & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: entity_type
  {
    size_t item_size = sizeof(ros_message.entity_type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.name.size() + 1);
  // Member: types
  {
    size_t array_size = ros_message.types.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (ros_message.types[index].size() + 1);
    }
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_micro_ros_msgs
max_serialized_size_Entity(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: entity_type
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: name
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        256 +
        1;
    }
  }

  // Member: types
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        256 +
        1;
    }
  }

  return current_alignment - initial_alignment;
}

static bool _Entity__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const micro_ros_msgs::msg::Entity *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Entity__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<micro_ros_msgs::msg::Entity *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Entity__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const micro_ros_msgs::msg::Entity *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Entity__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_Entity(full_bounded, 0);
}

static message_type_support_callbacks_t _Entity__callbacks = {
  "micro_ros_msgs::msg",
  "Entity",
  _Entity__cdr_serialize,
  _Entity__cdr_deserialize,
  _Entity__get_serialized_size,
  _Entity__max_serialized_size
};

static rosidl_message_type_support_t _Entity__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Entity__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace micro_ros_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_micro_ros_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<micro_ros_msgs::msg::Entity>()
{
  return &micro_ros_msgs::msg::typesupport_fastrtps_cpp::_Entity__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, micro_ros_msgs, msg, Entity)() {
  return &micro_ros_msgs::msg::typesupport_fastrtps_cpp::_Entity__handle;
}

#ifdef __cplusplus
}
#endif
