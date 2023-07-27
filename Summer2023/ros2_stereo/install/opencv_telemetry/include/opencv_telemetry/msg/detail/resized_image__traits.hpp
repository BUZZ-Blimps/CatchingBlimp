// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from opencv_telemetry:msg/ResizedImage.idl
// generated code does not contain a copyright notice

#ifndef OPENCV_TELEMETRY__MSG__DETAIL__RESIZED_IMAGE__TRAITS_HPP_
#define OPENCV_TELEMETRY__MSG__DETAIL__RESIZED_IMAGE__TRAITS_HPP_

#include "opencv_telemetry/msg/detail/resized_image__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'image'
#include "sensor_msgs/msg/detail/image__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<opencv_telemetry::msg::ResizedImage>()
{
  return "opencv_telemetry::msg::ResizedImage";
}

template<>
inline const char * name<opencv_telemetry::msg::ResizedImage>()
{
  return "opencv_telemetry/msg/ResizedImage";
}

template<>
struct has_fixed_size<opencv_telemetry::msg::ResizedImage>
  : std::integral_constant<bool, has_fixed_size<sensor_msgs::msg::Image>::value> {};

template<>
struct has_bounded_size<opencv_telemetry::msg::ResizedImage>
  : std::integral_constant<bool, has_bounded_size<sensor_msgs::msg::Image>::value> {};

template<>
struct is_message<opencv_telemetry::msg::ResizedImage>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // OPENCV_TELEMETRY__MSG__DETAIL__RESIZED_IMAGE__TRAITS_HPP_
