// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from micro_ros_msgs:msg/Entity.idl
// generated code does not contain a copyright notice

#ifndef MICRO_ROS_MSGS__MSG__DETAIL__ENTITY__TRAITS_HPP_
#define MICRO_ROS_MSGS__MSG__DETAIL__ENTITY__TRAITS_HPP_

#include "micro_ros_msgs/msg/detail/entity__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<micro_ros_msgs::msg::Entity>()
{
  return "micro_ros_msgs::msg::Entity";
}

template<>
inline const char * name<micro_ros_msgs::msg::Entity>()
{
  return "micro_ros_msgs/msg/Entity";
}

template<>
struct has_fixed_size<micro_ros_msgs::msg::Entity>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<micro_ros_msgs::msg::Entity>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<micro_ros_msgs::msg::Entity>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MICRO_ROS_MSGS__MSG__DETAIL__ENTITY__TRAITS_HPP_
