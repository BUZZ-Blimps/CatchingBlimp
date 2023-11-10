// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from yolo_msgs:msg/BoundingBoxes.idl
// generated code does not contain a copyright notice

#ifndef YOLO_MSGS__MSG__DETAIL__BOUNDING_BOXES__BUILDER_HPP_
#define YOLO_MSGS__MSG__DETAIL__BOUNDING_BOXES__BUILDER_HPP_

#include "yolo_msgs/msg/detail/bounding_boxes__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace yolo_msgs
{

namespace msg
{

namespace builder
{

class Init_BoundingBoxes_bounding_boxes
{
public:
  Init_BoundingBoxes_bounding_boxes()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::yolo_msgs::msg::BoundingBoxes bounding_boxes(::yolo_msgs::msg::BoundingBoxes::_bounding_boxes_type arg)
  {
    msg_.bounding_boxes = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yolo_msgs::msg::BoundingBoxes msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::yolo_msgs::msg::BoundingBoxes>()
{
  return yolo_msgs::msg::builder::Init_BoundingBoxes_bounding_boxes();
}

}  // namespace yolo_msgs

#endif  // YOLO_MSGS__MSG__DETAIL__BOUNDING_BOXES__BUILDER_HPP_
