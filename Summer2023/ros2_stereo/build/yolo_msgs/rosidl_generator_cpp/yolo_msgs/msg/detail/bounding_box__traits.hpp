// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from yolo_msgs:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef YOLO_MSGS__MSG__DETAIL__BOUNDING_BOX__TRAITS_HPP_
#define YOLO_MSGS__MSG__DETAIL__BOUNDING_BOX__TRAITS_HPP_

#include "yolo_msgs/msg/detail/bounding_box__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<yolo_msgs::msg::BoundingBox>()
{
  return "yolo_msgs::msg::BoundingBox";
}

template<>
inline const char * name<yolo_msgs::msg::BoundingBox>()
{
  return "yolo_msgs/msg/BoundingBox";
}

template<>
struct has_fixed_size<yolo_msgs::msg::BoundingBox>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<yolo_msgs::msg::BoundingBox>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<yolo_msgs::msg::BoundingBox>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // YOLO_MSGS__MSG__DETAIL__BOUNDING_BOX__TRAITS_HPP_
