// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from yolo_msgs:msg/ObjectCount.idl
// generated code does not contain a copyright notice
#include "yolo_msgs/msg/detail/object_count__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
yolo_msgs__msg__ObjectCount__init(yolo_msgs__msg__ObjectCount * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    yolo_msgs__msg__ObjectCount__fini(msg);
    return false;
  }
  // count
  return true;
}

void
yolo_msgs__msg__ObjectCount__fini(yolo_msgs__msg__ObjectCount * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // count
}

bool
yolo_msgs__msg__ObjectCount__are_equal(const yolo_msgs__msg__ObjectCount * lhs, const yolo_msgs__msg__ObjectCount * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // count
  if (lhs->count != rhs->count) {
    return false;
  }
  return true;
}

bool
yolo_msgs__msg__ObjectCount__copy(
  const yolo_msgs__msg__ObjectCount * input,
  yolo_msgs__msg__ObjectCount * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // count
  output->count = input->count;
  return true;
}

yolo_msgs__msg__ObjectCount *
yolo_msgs__msg__ObjectCount__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo_msgs__msg__ObjectCount * msg = (yolo_msgs__msg__ObjectCount *)allocator.allocate(sizeof(yolo_msgs__msg__ObjectCount), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(yolo_msgs__msg__ObjectCount));
  bool success = yolo_msgs__msg__ObjectCount__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
yolo_msgs__msg__ObjectCount__destroy(yolo_msgs__msg__ObjectCount * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    yolo_msgs__msg__ObjectCount__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
yolo_msgs__msg__ObjectCount__Sequence__init(yolo_msgs__msg__ObjectCount__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo_msgs__msg__ObjectCount * data = NULL;

  if (size) {
    data = (yolo_msgs__msg__ObjectCount *)allocator.zero_allocate(size, sizeof(yolo_msgs__msg__ObjectCount), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = yolo_msgs__msg__ObjectCount__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        yolo_msgs__msg__ObjectCount__fini(&data[i - 1]);
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
yolo_msgs__msg__ObjectCount__Sequence__fini(yolo_msgs__msg__ObjectCount__Sequence * array)
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
      yolo_msgs__msg__ObjectCount__fini(&array->data[i]);
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

yolo_msgs__msg__ObjectCount__Sequence *
yolo_msgs__msg__ObjectCount__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo_msgs__msg__ObjectCount__Sequence * array = (yolo_msgs__msg__ObjectCount__Sequence *)allocator.allocate(sizeof(yolo_msgs__msg__ObjectCount__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = yolo_msgs__msg__ObjectCount__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
yolo_msgs__msg__ObjectCount__Sequence__destroy(yolo_msgs__msg__ObjectCount__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    yolo_msgs__msg__ObjectCount__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
yolo_msgs__msg__ObjectCount__Sequence__are_equal(const yolo_msgs__msg__ObjectCount__Sequence * lhs, const yolo_msgs__msg__ObjectCount__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!yolo_msgs__msg__ObjectCount__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
yolo_msgs__msg__ObjectCount__Sequence__copy(
  const yolo_msgs__msg__ObjectCount__Sequence * input,
  yolo_msgs__msg__ObjectCount__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(yolo_msgs__msg__ObjectCount);
    yolo_msgs__msg__ObjectCount * data =
      (yolo_msgs__msg__ObjectCount *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!yolo_msgs__msg__ObjectCount__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          yolo_msgs__msg__ObjectCount__fini(&data[i]);
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
    if (!yolo_msgs__msg__ObjectCount__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
