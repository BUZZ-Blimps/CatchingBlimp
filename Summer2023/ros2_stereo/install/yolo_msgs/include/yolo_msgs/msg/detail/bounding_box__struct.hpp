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
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->probability = 0.0;
      this->x_center = 0ll;
      this->y_center = 0ll;
      this->width = 0ll;
      this->height = 0ll;
      this->track_id = 0;
      this->class_id = 0;
    }
  }

  explicit BoundingBox_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->probability = 0.0;
      this->x_center = 0ll;
      this->y_center = 0ll;
      this->width = 0ll;
      this->height = 0ll;
      this->track_id = 0;
      this->class_id = 0;
    }
  }

  // field types and members
  using _probability_type =
    double;
  _probability_type probability;
  using _x_center_type =
    int64_t;
  _x_center_type x_center;
  using _y_center_type =
    int64_t;
  _y_center_type y_center;
  using _width_type =
    int64_t;
  _width_type width;
  using _height_type =
    int64_t;
  _height_type height;
  using _track_id_type =
    int16_t;
  _track_id_type track_id;
  using _class_id_type =
    int16_t;
  _class_id_type class_id;

  // setters for named parameter idiom
  Type & set__probability(
    const double & _arg)
  {
    this->probability = _arg;
    return *this;
  }
  Type & set__x_center(
    const int64_t & _arg)
  {
    this->x_center = _arg;
    return *this;
  }
  Type & set__y_center(
    const int64_t & _arg)
  {
    this->y_center = _arg;
    return *this;
  }
  Type & set__width(
    const int64_t & _arg)
  {
    this->width = _arg;
    return *this;
  }
  Type & set__height(
    const int64_t & _arg)
  {
    this->height = _arg;
    return *this;
  }
  Type & set__track_id(
    const int16_t & _arg)
  {
    this->track_id = _arg;
    return *this;
  }
  Type & set__class_id(
    const int16_t & _arg)
  {
    this->class_id = _arg;
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
    if (this->probability != other.probability) {
      return false;
    }
    if (this->x_center != other.x_center) {
      return false;
    }
    if (this->y_center != other.y_center) {
      return false;
    }
    if (this->width != other.width) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->track_id != other.track_id) {
      return false;
    }
    if (this->class_id != other.class_id) {
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
