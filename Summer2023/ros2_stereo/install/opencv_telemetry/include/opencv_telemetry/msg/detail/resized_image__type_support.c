// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from opencv_telemetry:msg/ResizedImage.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "opencv_telemetry/msg/detail/resized_image__rosidl_typesupport_introspection_c.h"
#include "opencv_telemetry/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "opencv_telemetry/msg/detail/resized_image__functions.h"
#include "opencv_telemetry/msg/detail/resized_image__struct.h"


// Include directives for member types
// Member `image`
#include "sensor_msgs/msg/image.h"
// Member `image`
#include "sensor_msgs/msg/detail/image__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ResizedImage__rosidl_typesupport_introspection_c__ResizedImage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  opencv_telemetry__msg__ResizedImage__init(message_memory);
}

void ResizedImage__rosidl_typesupport_introspection_c__ResizedImage_fini_function(void * message_memory)
{
  opencv_telemetry__msg__ResizedImage__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ResizedImage__rosidl_typesupport_introspection_c__ResizedImage_message_member_array[3] = {
  {
    "original_height",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(opencv_telemetry__msg__ResizedImage, original_height),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "original_width",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(opencv_telemetry__msg__ResizedImage, original_width),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "image",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(opencv_telemetry__msg__ResizedImage, image),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ResizedImage__rosidl_typesupport_introspection_c__ResizedImage_message_members = {
  "opencv_telemetry__msg",  // message namespace
  "ResizedImage",  // message name
  3,  // number of fields
  sizeof(opencv_telemetry__msg__ResizedImage),
  ResizedImage__rosidl_typesupport_introspection_c__ResizedImage_message_member_array,  // message members
  ResizedImage__rosidl_typesupport_introspection_c__ResizedImage_init_function,  // function to initialize message memory (memory has to be allocated)
  ResizedImage__rosidl_typesupport_introspection_c__ResizedImage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ResizedImage__rosidl_typesupport_introspection_c__ResizedImage_message_type_support_handle = {
  0,
  &ResizedImage__rosidl_typesupport_introspection_c__ResizedImage_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_opencv_telemetry
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, opencv_telemetry, msg, ResizedImage)() {
  ResizedImage__rosidl_typesupport_introspection_c__ResizedImage_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, Image)();
  if (!ResizedImage__rosidl_typesupport_introspection_c__ResizedImage_message_type_support_handle.typesupport_identifier) {
    ResizedImage__rosidl_typesupport_introspection_c__ResizedImage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ResizedImage__rosidl_typesupport_introspection_c__ResizedImage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
