// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from yolo_msgs:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef YOLO_MSGS__MSG__DETAIL__BOUNDING_BOX__BUILDER_HPP_
#define YOLO_MSGS__MSG__DETAIL__BOUNDING_BOX__BUILDER_HPP_

#include "yolo_msgs/msg/detail/bounding_box__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace yolo_msgs
{

namespace msg
{

namespace builder
{

class Init_BoundingBox_class_id
{
public:
  explicit Init_BoundingBox_class_id(::yolo_msgs::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  ::yolo_msgs::msg::BoundingBox class_id(::yolo_msgs::msg::BoundingBox::_class_id_type arg)
  {
    msg_.class_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yolo_msgs::msg::BoundingBox msg_;
};

class Init_BoundingBox_track_id
{
public:
  explicit Init_BoundingBox_track_id(::yolo_msgs::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_class_id track_id(::yolo_msgs::msg::BoundingBox::_track_id_type arg)
  {
    msg_.track_id = std::move(arg);
    return Init_BoundingBox_class_id(msg_);
  }

private:
  ::yolo_msgs::msg::BoundingBox msg_;
};

class Init_BoundingBox_height
{
public:
  explicit Init_BoundingBox_height(::yolo_msgs::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_track_id height(::yolo_msgs::msg::BoundingBox::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_BoundingBox_track_id(msg_);
  }

private:
  ::yolo_msgs::msg::BoundingBox msg_;
};

class Init_BoundingBox_width
{
public:
  explicit Init_BoundingBox_width(::yolo_msgs::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_height width(::yolo_msgs::msg::BoundingBox::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_BoundingBox_height(msg_);
  }

private:
  ::yolo_msgs::msg::BoundingBox msg_;
};

class Init_BoundingBox_y_center
{
public:
  explicit Init_BoundingBox_y_center(::yolo_msgs::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_width y_center(::yolo_msgs::msg::BoundingBox::_y_center_type arg)
  {
    msg_.y_center = std::move(arg);
    return Init_BoundingBox_width(msg_);
  }

private:
  ::yolo_msgs::msg::BoundingBox msg_;
};

class Init_BoundingBox_x_center
{
public:
  explicit Init_BoundingBox_x_center(::yolo_msgs::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_y_center x_center(::yolo_msgs::msg::BoundingBox::_x_center_type arg)
  {
    msg_.x_center = std::move(arg);
    return Init_BoundingBox_y_center(msg_);
  }

private:
  ::yolo_msgs::msg::BoundingBox msg_;
};

class Init_BoundingBox_probability
{
public:
  Init_BoundingBox_probability()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BoundingBox_x_center probability(::yolo_msgs::msg::BoundingBox::_probability_type arg)
  {
    msg_.probability = std::move(arg);
    return Init_BoundingBox_x_center(msg_);
  }

private:
  ::yolo_msgs::msg::BoundingBox msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::yolo_msgs::msg::BoundingBox>()
{
  return yolo_msgs::msg::builder::Init_BoundingBox_probability();
}

}  // namespace yolo_msgs

#endif  // YOLO_MSGS__MSG__DETAIL__BOUNDING_BOX__BUILDER_HPP_
