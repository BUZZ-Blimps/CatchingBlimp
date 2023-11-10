// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from yolo_msgs:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef YOLO_MSGS__MSG__DETAIL__BOUNDING_BOX__STRUCT_HPP_
#define YOLO_MSGS__MSG__DETAIL__BOUNDING_BOX__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__yolo_msgs__msg__BoundingBox __attribute__((deprecated))
#else
# define DEPRECATED__yolo_msgs__msg__BoundingBox __declspec(deprecated)
#endif

namespace yolo_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BoundingBox_
{
  using Type = BoundingBox_<ContainerAllocator>;

  explicit BoundingBox_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x_center_balloon = 0ll;
      this->y_center_balloon = 0ll;
      this->width_balloon = 0ll;
      this->height_balloon = 0ll;
      this->x_center_y_goal = 0ll;
      this->y_center_y_goal = 0ll;
      this->width_y_goal = 0ll;
      this->height_y_goal = 0ll;
      this->x_center_o_goal = 0ll;
      this->y_center_o_goal = 0ll;
      this->width_o_goal = 0ll;
      this->height_o_goal = 0ll;
    }
  }

  explicit BoundingBox_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x_center_balloon = 0ll;
      this->y_center_balloon = 0ll;
      this->width_balloon = 0ll;
      this->height_balloon = 0ll;
      this->x_center_y_goal = 0ll;
      this->y_center_y_goal = 0ll;
      this->width_y_goal = 0ll;
      this->height_y_goal = 0ll;
      this->x_center_o_goal = 0ll;
      this->y_center_o_goal = 0ll;
      this->width_o_goal = 0ll;
      this->height_o_goal = 0ll;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _x_center_balloon_type =
    int64_t;
  _x_center_balloon_type x_center_balloon;
  using _y_center_balloon_type =
    int64_t;
  _y_center_balloon_type y_center_balloon;
  using _width_balloon_type =
    int64_t;
  _width_balloon_type width_balloon;
  using _height_balloon_type =
    int64_t;
  _height_balloon_type height_balloon;
  using _x_center_y_goal_type =
    int64_t;
  _x_center_y_goal_type x_center_y_goal;
  using _y_center_y_goal_type =
    int64_t;
  _y_center_y_goal_type y_center_y_goal;
  using _width_y_goal_type =
    int64_t;
  _width_y_goal_type width_y_goal;
  using _height_y_goal_type =
    int64_t;
  _height_y_goal_type height_y_goal;
  using _x_center_o_goal_type =
    int64_t;
  _x_center_o_goal_type x_center_o_goal;
  using _y_center_o_goal_type =
    int64_t;
  _y_center_o_goal_type y_center_o_goal;
  using _width_o_goal_type =
    int64_t;
  _width_o_goal_type width_o_goal;
  using _height_o_goal_type =
    int64_t;
  _height_o_goal_type height_o_goal;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__x_center_balloon(
    const int64_t & _arg)
  {
    this->x_center_balloon = _arg;
    return *this;
  }
  Type & set__y_center_balloon(
    const int64_t & _arg)
  {
    this->y_center_balloon = _arg;
    return *this;
  }
  Type & set__width_balloon(
    const int64_t & _arg)
  {
    this->width_balloon = _arg;
    return *this;
  }
  Type & set__height_balloon(
    const int64_t & _arg)
  {
    this->height_balloon = _arg;
    return *this;
  }
  Type & set__x_center_y_goal(
    const int64_t & _arg)
  {
    this->x_center_y_goal = _arg;
    return *this;
  }
  Type & set__y_center_y_goal(
    const int64_t & _arg)
  {
    this->y_center_y_goal = _arg;
    return *this;
  }
  Type & set__width_y_goal(
    const int64_t & _arg)
  {
    this->width_y_goal = _arg;
    return *this;
  }
  Type & set__height_y_goal(
    const int64_t & _arg)
  {
    this->height_y_goal = _arg;
    return *this;
  }
  Type & set__x_center_o_goal(
    const int64_t & _arg)
  {
    this->x_center_o_goal = _arg;
    return *this;
  }
  Type & set__y_center_o_goal(
    const int64_t & _arg)
  {
    this->y_center_o_goal = _arg;
    return *this;
  }
  Type & set__width_o_goal(
    const int64_t & _arg)
  {
    this->width_o_goal = _arg;
    return *this;
  }
  Type & set__height_o_goal(
    const int64_t & _arg)
  {
    this->height_o_goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    yolo_msgs::msg::BoundingBox_<ContainerAllocator> *;
  using ConstRawPtr =
    const yolo_msgs::msg::BoundingBox_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<yolo_msgs::msg::BoundingBox_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<yolo_msgs::msg::BoundingBox_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      yolo_msgs::msg::BoundingBox_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<yolo_msgs::msg::BoundingBox_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      yolo_msgs::msg::BoundingBox_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<yolo_msgs::msg::BoundingBox_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<yolo_msgs::msg::BoundingBox_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<yolo_msgs::msg::BoundingBox_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__yolo_msgs__msg__BoundingBox
    std::shared_ptr<yolo_msgs::msg::BoundingBox_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__yolo_msgs__msg__BoundingBox
    std::shared_ptr<yolo_msgs::msg::BoundingBox_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BoundingBox_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->x_center_balloon != other.x_center_balloon) {
      return false;
    }
    if (this->y_center_balloon != other.y_center_balloon) {
      return false;
    }
    if (this->width_balloon != other.width_balloon) {
      return false;
    }
    if (this->height_balloon != other.height_balloon) {
      return false;
    }
    if (this->x_center_y_goal != other.x_center_y_goal) {
      return false;
    }
    if (this->y_center_y_goal != other.y_center_y_goal) {
      return false;
    }
    if (this->width_y_goal != other.width_y_goal) {
      return false;
    }
    if (this->height_y_goal != other.height_y_goal) {
      return false;
    }
    if (this->x_center_o_goal != other.x_center_o_goal) {
      return false;
    }
    if (this->y_center_o_goal != other.y_center_o_goal) {
      return false;
    }
    if (this->width_o_goal != other.width_o_goal) {
      return false;
    }
    if (this->height_o_goal != other.height_o_goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const BoundingBox_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BoundingBox_

// alias to use template instance with default allocator
using BoundingBox =
  yolo_msgs::msg::BoundingBox_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace yolo_msgs

#endif  // YOLO_MSGS__MSG__DETAIL__BOUNDING_BOX__STRUCT_HPP_
