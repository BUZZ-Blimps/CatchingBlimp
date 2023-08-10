// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from yolo_msgs:msg/ObjectCount.idl
// generated code does not contain a copyright notice

#ifndef YOLO_MSGS__MSG__DETAIL__OBJECT_COUNT__BUILDER_HPP_
#define YOLO_MSGS__MSG__DETAIL__OBJECT_COUNT__BUILDER_HPP_

#include "yolo_msgs/msg/detail/object_count__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace yolo_msgs
{

namespace msg
{

namespace builder
{

class Init_ObjectCount_count
{
public:
  explicit Init_ObjectCount_count(::yolo_msgs::msg::ObjectCount & msg)
  : msg_(msg)
  {}
  ::yolo_msgs::msg::ObjectCount count(::yolo_msgs::msg::ObjectCount::_count_type arg)
  {
    msg_.count = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yolo_msgs::msg::ObjectCount msg_;
};

class Init_ObjectCount_header
{
public:
  Init_ObjectCount_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObjectCount_count header(::yolo_msgs::msg::ObjectCount::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ObjectCount_count(msg_);
  }

private:
  ::yolo_msgs::msg::ObjectCount msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::yolo_msgs::msg::ObjectCount>()
{
  return yolo_msgs::msg::builder::Init_ObjectCount_header();
}

}  // namespace yolo_msgs

#endif  // YOLO_MSGS__MSG__DETAIL__OBJECT_COUNT__BUILDER_HPP_
