// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from opencv_telemetry:msg/ResizedImage.idl
// generated code does not contain a copyright notice

#ifndef OPENCV_TELEMETRY__MSG__DETAIL__RESIZED_IMAGE__BUILDER_HPP_
#define OPENCV_TELEMETRY__MSG__DETAIL__RESIZED_IMAGE__BUILDER_HPP_

#include "opencv_telemetry/msg/detail/resized_image__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace opencv_telemetry
{

namespace msg
{

namespace builder
{

class Init_ResizedImage_image
{
public:
  explicit Init_ResizedImage_image(::opencv_telemetry::msg::ResizedImage & msg)
  : msg_(msg)
  {}
  ::opencv_telemetry::msg::ResizedImage image(::opencv_telemetry::msg::ResizedImage::_image_type arg)
  {
    msg_.image = std::move(arg);
    return std::move(msg_);
  }

private:
  ::opencv_telemetry::msg::ResizedImage msg_;
};

class Init_ResizedImage_original_width
{
public:
  explicit Init_ResizedImage_original_width(::opencv_telemetry::msg::ResizedImage & msg)
  : msg_(msg)
  {}
  Init_ResizedImage_image original_width(::opencv_telemetry::msg::ResizedImage::_original_width_type arg)
  {
    msg_.original_width = std::move(arg);
    return Init_ResizedImage_image(msg_);
  }

private:
  ::opencv_telemetry::msg::ResizedImage msg_;
};

class Init_ResizedImage_original_height
{
public:
  Init_ResizedImage_original_height()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ResizedImage_original_width original_height(::opencv_telemetry::msg::ResizedImage::_original_height_type arg)
  {
    msg_.original_height = std::move(arg);
    return Init_ResizedImage_original_width(msg_);
  }

private:
  ::opencv_telemetry::msg::ResizedImage msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::opencv_telemetry::msg::ResizedImage>()
{
  return opencv_telemetry::msg::builder::Init_ResizedImage_original_height();
}

}  // namespace opencv_telemetry

#endif  // OPENCV_TELEMETRY__MSG__DETAIL__RESIZED_IMAGE__BUILDER_HPP_
