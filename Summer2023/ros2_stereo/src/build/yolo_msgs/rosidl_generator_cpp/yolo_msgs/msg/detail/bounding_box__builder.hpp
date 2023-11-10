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

class Init_BoundingBox_height_o_goal
{
public:
  explicit Init_BoundingBox_height_o_goal(::yolo_msgs::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  ::yolo_msgs::msg::BoundingBox height_o_goal(::yolo_msgs::msg::BoundingBox::_height_o_goal_type arg)
  {
    msg_.height_o_goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yolo_msgs::msg::BoundingBox msg_;
};

class Init_BoundingBox_width_o_goal
{
public:
  explicit Init_BoundingBox_width_o_goal(::yolo_msgs::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_height_o_goal width_o_goal(::yolo_msgs::msg::BoundingBox::_width_o_goal_type arg)
  {
    msg_.width_o_goal = std::move(arg);
    return Init_BoundingBox_height_o_goal(msg_);
  }

private:
  ::yolo_msgs::msg::BoundingBox msg_;
};

class Init_BoundingBox_y_center_o_goal
{
public:
  explicit Init_BoundingBox_y_center_o_goal(::yolo_msgs::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_width_o_goal y_center_o_goal(::yolo_msgs::msg::BoundingBox::_y_center_o_goal_type arg)
  {
    msg_.y_center_o_goal = std::move(arg);
    return Init_BoundingBox_width_o_goal(msg_);
  }

private:
  ::yolo_msgs::msg::BoundingBox msg_;
};

class Init_BoundingBox_x_center_o_goal
{
public:
  explicit Init_BoundingBox_x_center_o_goal(::yolo_msgs::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_y_center_o_goal x_center_o_goal(::yolo_msgs::msg::BoundingBox::_x_center_o_goal_type arg)
  {
    msg_.x_center_o_goal = std::move(arg);
    return Init_BoundingBox_y_center_o_goal(msg_);
  }

private:
  ::yolo_msgs::msg::BoundingBox msg_;
};

class Init_BoundingBox_height_y_goal
{
public:
  explicit Init_BoundingBox_height_y_goal(::yolo_msgs::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_x_center_o_goal height_y_goal(::yolo_msgs::msg::BoundingBox::_height_y_goal_type arg)
  {
    msg_.height_y_goal = std::move(arg);
    return Init_BoundingBox_x_center_o_goal(msg_);
  }

private:
  ::yolo_msgs::msg::BoundingBox msg_;
};

class Init_BoundingBox_width_y_goal
{
public:
  explicit Init_BoundingBox_width_y_goal(::yolo_msgs::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_height_y_goal width_y_goal(::yolo_msgs::msg::BoundingBox::_width_y_goal_type arg)
  {
    msg_.width_y_goal = std::move(arg);
    return Init_BoundingBox_height_y_goal(msg_);
  }

private:
  ::yolo_msgs::msg::BoundingBox msg_;
};

class Init_BoundingBox_y_center_y_goal
{
public:
  explicit Init_BoundingBox_y_center_y_goal(::yolo_msgs::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_width_y_goal y_center_y_goal(::yolo_msgs::msg::BoundingBox::_y_center_y_goal_type arg)
  {
    msg_.y_center_y_goal = std::move(arg);
    return Init_BoundingBox_width_y_goal(msg_);
  }

private:
  ::yolo_msgs::msg::BoundingBox msg_;
};

class Init_BoundingBox_x_center_y_goal
{
public:
  explicit Init_BoundingBox_x_center_y_goal(::yolo_msgs::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_y_center_y_goal x_center_y_goal(::yolo_msgs::msg::BoundingBox::_x_center_y_goal_type arg)
  {
    msg_.x_center_y_goal = std::move(arg);
    return Init_BoundingBox_y_center_y_goal(msg_);
  }

private:
  ::yolo_msgs::msg::BoundingBox msg_;
};

class Init_BoundingBox_height_balloon
{
public:
  explicit Init_BoundingBox_height_balloon(::yolo_msgs::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_x_center_y_goal height_balloon(::yolo_msgs::msg::BoundingBox::_height_balloon_type arg)
  {
    msg_.height_balloon = std::move(arg);
    return Init_BoundingBox_x_center_y_goal(msg_);
  }

private:
  ::yolo_msgs::msg::BoundingBox msg_;
};

class Init_BoundingBox_width_balloon
{
public:
  explicit Init_BoundingBox_width_balloon(::yolo_msgs::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_height_balloon width_balloon(::yolo_msgs::msg::BoundingBox::_width_balloon_type arg)
  {
    msg_.width_balloon = std::move(arg);
    return Init_BoundingBox_height_balloon(msg_);
  }

private:
  ::yolo_msgs::msg::BoundingBox msg_;
};

class Init_BoundingBox_y_center_balloon
{
public:
  explicit Init_BoundingBox_y_center_balloon(::yolo_msgs::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_width_balloon y_center_balloon(::yolo_msgs::msg::BoundingBox::_y_center_balloon_type arg)
  {
    msg_.y_center_balloon = std::move(arg);
    return Init_BoundingBox_width_balloon(msg_);
  }

private:
  ::yolo_msgs::msg::BoundingBox msg_;
};

class Init_BoundingBox_x_center_balloon
{
public:
  explicit Init_BoundingBox_x_center_balloon(::yolo_msgs::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_y_center_balloon x_center_balloon(::yolo_msgs::msg::BoundingBox::_x_center_balloon_type arg)
  {
    msg_.x_center_balloon = std::move(arg);
    return Init_BoundingBox_y_center_balloon(msg_);
  }

private:
  ::yolo_msgs::msg::BoundingBox msg_;
};

class Init_BoundingBox_header
{
public:
  Init_BoundingBox_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BoundingBox_x_center_balloon header(::yolo_msgs::msg::BoundingBox::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_BoundingBox_x_center_balloon(msg_);
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
  return yolo_msgs::msg::builder::Init_BoundingBox_header();
}

}  // namespace yolo_msgs

#endif  // YOLO_MSGS__MSG__DETAIL__BOUNDING_BOX__BUILDER_HPP_
