// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from micro_ros_msgs:msg/Graph.idl
// generated code does not contain a copyright notice

#ifndef MICRO_ROS_MSGS__MSG__DETAIL__GRAPH__TRAITS_HPP_
#define MICRO_ROS_MSGS__MSG__DETAIL__GRAPH__TRAITS_HPP_

#include "micro_ros_msgs/msg/detail/graph__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<micro_ros_msgs::msg::Graph>()
{
  return "micro_ros_msgs::msg::Graph";
}

template<>
inline const char * name<micro_ros_msgs::msg::Graph>()
{
  return "micro_ros_msgs/msg/Graph";
}

template<>
struct has_fixed_size<micro_ros_msgs::msg::Graph>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<micro_ros_msgs::msg::Graph>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<micro_ros_msgs::msg::Graph>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MICRO_ROS_MSGS__MSG__DETAIL__GRAPH__TRAITS_HPP_
