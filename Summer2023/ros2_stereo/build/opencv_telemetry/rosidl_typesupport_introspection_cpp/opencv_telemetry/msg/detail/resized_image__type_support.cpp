// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from opencv_telemetry:msg/ResizedImage.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "opencv_telemetry/msg/detail/resized_image__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace opencv_telemetry
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ResizedImage_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) opencv_telemetry::msg::ResizedImage(_init);
}

void ResizedImage_fini_function(void * message_memory)
{
  auto typed_message = static_cast<opencv_telemetry::msg::ResizedImage *>(message_memory);
  typed_message->~ResizedImage();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ResizedImage_message_member_array[3] = {
  {
    "original_height",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(opencv_telemetry::msg::ResizedImage, original_height),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "original_width",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(opencv_telemetry::msg::ResizedImage, original_width),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "image",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<sensor_msgs::msg::Image>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(opencv_telemetry::msg::ResizedImage, image),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ResizedImage_message_members = {
  "opencv_telemetry::msg",  // message namespace
  "ResizedImage",  // message name
  3,  // number of fields
  sizeof(opencv_telemetry::msg::ResizedImage),
  ResizedImage_message_member_array,  // message members
  ResizedImage_init_function,  // function to initialize message memory (memory has to be allocated)
  ResizedImage_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ResizedImage_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ResizedImage_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace opencv_telemetry


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<opencv_telemetry::msg::ResizedImage>()
{
  return &::opencv_telemetry::msg::rosidl_typesupport_introspection_cpp::ResizedImage_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, opencv_telemetry, msg, ResizedImage)() {
  return &::opencv_telemetry::msg::rosidl_typesupport_introspection_cpp::ResizedImage_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
