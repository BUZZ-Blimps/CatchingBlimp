// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from yolo_msgs:msg/ObjectCount.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "yolo_msgs/msg/detail/object_count__rosidl_typesupport_introspection_c.h"
#include "yolo_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "yolo_msgs/msg/detail/object_count__functions.h"
#include "yolo_msgs/msg/detail/object_count__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ObjectCount__rosidl_typesupport_introspection_c__ObjectCount_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  yolo_msgs__msg__ObjectCount__init(message_memory);
}

void ObjectCount__rosidl_typesupport_introspection_c__ObjectCount_fini_function(void * message_memory)
{
  yolo_msgs__msg__ObjectCount__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ObjectCount__rosidl_typesupport_introspection_c__ObjectCount_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolo_msgs__msg__ObjectCount, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "count",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(yolo_msgs__msg__ObjectCount, count),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ObjectCount__rosidl_typesupport_introspection_c__ObjectCount_message_members = {
  "yolo_msgs__msg",  // message namespace
  "ObjectCount",  // message name
  2,  // number of fields
  sizeof(yolo_msgs__msg__ObjectCount),
  ObjectCount__rosidl_typesupport_introspection_c__ObjectCount_message_member_array,  // message members
  ObjectCount__rosidl_typesupport_introspection_c__ObjectCount_init_function,  // function to initialize message memory (memory has to be allocated)
  ObjectCount__rosidl_typesupport_introspection_c__ObjectCount_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ObjectCount__rosidl_typesupport_introspection_c__ObjectCount_message_type_support_handle = {
  0,
  &ObjectCount__rosidl_typesupport_introspection_c__ObjectCount_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_yolo_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, yolo_msgs, msg, ObjectCount)() {
  ObjectCount__rosidl_typesupport_introspection_c__ObjectCount_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!ObjectCount__rosidl_typesupport_introspection_c__ObjectCount_message_type_support_handle.typesupport_identifier) {
    ObjectCount__rosidl_typesupport_introspection_c__ObjectCount_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ObjectCount__rosidl_typesupport_introspection_c__ObjectCount_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
