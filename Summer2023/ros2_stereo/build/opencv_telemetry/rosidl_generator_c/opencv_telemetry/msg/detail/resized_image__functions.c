// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from opencv_telemetry:msg/ResizedImage.idl
// generated code does not contain a copyright notice
#include "opencv_telemetry/msg/detail/resized_image__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `image`
#include "sensor_msgs/msg/detail/image__functions.h"

bool
opencv_telemetry__msg__ResizedImage__init(opencv_telemetry__msg__ResizedImage * msg)
{
  if (!msg) {
    return false;
  }
  // original_height
  // original_width
  // image
  if (!sensor_msgs__msg__Image__init(&msg->image)) {
    opencv_telemetry__msg__ResizedImage__fini(msg);
    return false;
  }
  return true;
}

void
opencv_telemetry__msg__ResizedImage__fini(opencv_telemetry__msg__ResizedImage * msg)
{
  if (!msg) {
    return;
  }
  // original_height
  // original_width
  // image
  sensor_msgs__msg__Image__fini(&msg->image);
}

bool
opencv_telemetry__msg__ResizedImage__are_equal(const opencv_telemetry__msg__ResizedImage * lhs, const opencv_telemetry__msg__ResizedImage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // original_height
  if (lhs->original_height != rhs->original_height) {
    return false;
  }
  // original_width
  if (lhs->original_width != rhs->original_width) {
    return false;
  }
  // image
  if (!sensor_msgs__msg__Image__are_equal(
      &(lhs->image), &(rhs->image)))
  {
    return false;
  }
  return true;
}

bool
opencv_telemetry__msg__ResizedImage__copy(
  const opencv_telemetry__msg__ResizedImage * input,
  opencv_telemetry__msg__ResizedImage * output)
{
  if (!input || !output) {
    return false;
  }
  // original_height
  output->original_height = input->original_height;
  // original_width
  output->original_width = input->original_width;
  // image
  if (!sensor_msgs__msg__Image__copy(
      &(input->image), &(output->image)))
  {
    return false;
  }
  return true;
}

opencv_telemetry__msg__ResizedImage *
opencv_telemetry__msg__ResizedImage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  opencv_telemetry__msg__ResizedImage * msg = (opencv_telemetry__msg__ResizedImage *)allocator.allocate(sizeof(opencv_telemetry__msg__ResizedImage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(opencv_telemetry__msg__ResizedImage));
  bool success = opencv_telemetry__msg__ResizedImage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
opencv_telemetry__msg__ResizedImage__destroy(opencv_telemetry__msg__ResizedImage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    opencv_telemetry__msg__ResizedImage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
opencv_telemetry__msg__ResizedImage__Sequence__init(opencv_telemetry__msg__ResizedImage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  opencv_telemetry__msg__ResizedImage * data = NULL;

  if (size) {
    data = (opencv_telemetry__msg__ResizedImage *)allocator.zero_allocate(size, sizeof(opencv_telemetry__msg__ResizedImage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = opencv_telemetry__msg__ResizedImage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        opencv_telemetry__msg__ResizedImage__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
opencv_telemetry__msg__ResizedImage__Sequence__fini(opencv_telemetry__msg__ResizedImage__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      opencv_telemetry__msg__ResizedImage__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

opencv_telemetry__msg__ResizedImage__Sequence *
opencv_telemetry__msg__ResizedImage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  opencv_telemetry__msg__ResizedImage__Sequence * array = (opencv_telemetry__msg__ResizedImage__Sequence *)allocator.allocate(sizeof(opencv_telemetry__msg__ResizedImage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = opencv_telemetry__msg__ResizedImage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
opencv_telemetry__msg__ResizedImage__Sequence__destroy(opencv_telemetry__msg__ResizedImage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    opencv_telemetry__msg__ResizedImage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
opencv_telemetry__msg__ResizedImage__Sequence__are_equal(const opencv_telemetry__msg__ResizedImage__Sequence * lhs, const opencv_telemetry__msg__ResizedImage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!opencv_telemetry__msg__ResizedImage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
opencv_telemetry__msg__ResizedImage__Sequence__copy(
  const opencv_telemetry__msg__ResizedImage__Sequence * input,
  opencv_telemetry__msg__ResizedImage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(opencv_telemetry__msg__ResizedImage);
    opencv_telemetry__msg__ResizedImage * data =
      (opencv_telemetry__msg__ResizedImage *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!opencv_telemetry__msg__ResizedImage__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          opencv_telemetry__msg__ResizedImage__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!opencv_telemetry__msg__ResizedImage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
