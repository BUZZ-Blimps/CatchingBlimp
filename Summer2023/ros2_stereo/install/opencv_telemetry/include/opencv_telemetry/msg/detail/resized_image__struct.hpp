// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from opencv_telemetry:msg/ResizedImage.idl
// generated code does not contain a copyright notice

#ifndef OPENCV_TELEMETRY__MSG__DETAIL__RESIZED_IMAGE__STRUCT_HPP_
#define OPENCV_TELEMETRY__MSG__DETAIL__RESIZED_IMAGE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'image'
#include "sensor_msgs/msg/detail/image__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__opencv_telemetry__msg__ResizedImage __attribute__((deprecated))
#else
# define DEPRECATED__opencv_telemetry__msg__ResizedImage __declspec(deprecated)
#endif

namespace opencv_telemetry
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ResizedImage_
{
  using Type = ResizedImage_<ContainerAllocator>;

  explicit ResizedImage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : image(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->original_height = 0ul;
      this->original_width = 0ul;
    }
  }

  explicit ResizedImage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : image(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->original_height = 0ul;
      this->original_width = 0ul;
    }
  }

  // field types and members
  using _original_height_type =
    uint32_t;
  _original_height_type original_height;
  using _original_width_type =
    uint32_t;
  _original_width_type original_width;
  using _image_type =
    sensor_msgs::msg::Image_<ContainerAllocator>;
  _image_type image;

  // setters for named parameter idiom
  Type & set__original_height(
    const uint32_t & _arg)
  {
    this->original_height = _arg;
    return *this;
  }
  Type & set__original_width(
    const uint32_t & _arg)
  {
    this->original_width = _arg;
    return *this;
  }
  Type & set__image(
    const sensor_msgs::msg::Image_<ContainerAllocator> & _arg)
  {
    this->image = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    opencv_telemetry::msg::ResizedImage_<ContainerAllocator> *;
  using ConstRawPtr =
    const opencv_telemetry::msg::ResizedImage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<opencv_telemetry::msg::ResizedImage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<opencv_telemetry::msg::ResizedImage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      opencv_telemetry::msg::ResizedImage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<opencv_telemetry::msg::ResizedImage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      opencv_telemetry::msg::ResizedImage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<opencv_telemetry::msg::ResizedImage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<opencv_telemetry::msg::ResizedImage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<opencv_telemetry::msg::ResizedImage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__opencv_telemetry__msg__ResizedImage
    std::shared_ptr<opencv_telemetry::msg::ResizedImage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__opencv_telemetry__msg__ResizedImage
    std::shared_ptr<opencv_telemetry::msg::ResizedImage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ResizedImage_ & other) const
  {
    if (this->original_height != other.original_height) {
      return false;
    }
    if (this->original_width != other.original_width) {
      return false;
    }
    if (this->image != other.image) {
      return false;
    }
    return true;
  }
  bool operator!=(const ResizedImage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ResizedImage_

// alias to use template instance with default allocator
using ResizedImage =
  opencv_telemetry::msg::ResizedImage_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace opencv_telemetry

#endif  // OPENCV_TELEMETRY__MSG__DETAIL__RESIZED_IMAGE__STRUCT_HPP_
