// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from yolo_msgs:msg/BoundingBoxes.idl
// generated code does not contain a copyright notice
#include "yolo_msgs/msg/detail/bounding_boxes__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `bounding_boxes`
#include "yolo_msgs/msg/detail/bounding_box__functions.h"

bool
yolo_msgs__msg__BoundingBoxes__init(yolo_msgs__msg__BoundingBoxes * msg)
{
  if (!msg) {
    return false;
  }
  // bounding_boxes
  if (!yolo_msgs__msg__BoundingBox__Sequence__init(&msg->bounding_boxes, 0)) {
    yolo_msgs__msg__BoundingBoxes__fini(msg);
    return false;
  }
  return true;
}

void
yolo_msgs__msg__BoundingBoxes__fini(yolo_msgs__msg__BoundingBoxes * msg)
{
  if (!msg) {
    return;
  }
  // bounding_boxes
  yolo_msgs__msg__BoundingBox__Sequence__fini(&msg->bounding_boxes);
}

bool
yolo_msgs__msg__BoundingBoxes__are_equal(const yolo_msgs__msg__BoundingBoxes * lhs, const yolo_msgs__msg__BoundingBoxes * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // bounding_boxes
  if (!yolo_msgs__msg__BoundingBox__Sequence__are_equal(
      &(lhs->bounding_boxes), &(rhs->bounding_boxes)))
  {
    return false;
  }
  return true;
}

bool
yolo_msgs__msg__BoundingBoxes__copy(
  const yolo_msgs__msg__BoundingBoxes * input,
  yolo_msgs__msg__BoundingBoxes * output)
{
  if (!input || !output) {
    return false;
  }
  // bounding_boxes
  if (!yolo_msgs__msg__BoundingBox__Sequence__copy(
      &(input->bounding_boxes), &(output->bounding_boxes)))
  {
    return false;
  }
  return true;
}

yolo_msgs__msg__BoundingBoxes *
yolo_msgs__msg__BoundingBoxes__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo_msgs__msg__BoundingBoxes * msg = (yolo_msgs__msg__BoundingBoxes *)allocator.allocate(sizeof(yolo_msgs__msg__BoundingBoxes), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(yolo_msgs__msg__BoundingBoxes));
  bool success = yolo_msgs__msg__BoundingBoxes__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
yolo_msgs__msg__BoundingBoxes__destroy(yolo_msgs__msg__BoundingBoxes * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    yolo_msgs__msg__BoundingBoxes__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
yolo_msgs__msg__BoundingBoxes__Sequence__init(yolo_msgs__msg__BoundingBoxes__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo_msgs__msg__BoundingBoxes * data = NULL;

  if (size) {
    data = (yolo_msgs__msg__BoundingBoxes *)allocator.zero_allocate(size, sizeof(yolo_msgs__msg__BoundingBoxes), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = yolo_msgs__msg__BoundingBoxes__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        yolo_msgs__msg__BoundingBoxes__fini(&data[i - 1]);
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
yolo_msgs__msg__BoundingBoxes__Sequence__fini(yolo_msgs__msg__BoundingBoxes__Sequence * array)
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
      yolo_msgs__msg__BoundingBoxes__fini(&array->data[i]);
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

yolo_msgs__msg__BoundingBoxes__Sequence *
yolo_msgs__msg__BoundingBoxes__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo_msgs__msg__BoundingBoxes__Sequence * array = (yolo_msgs__msg__BoundingBoxes__Sequence *)allocator.allocate(sizeof(yolo_msgs__msg__BoundingBoxes__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = yolo_msgs__msg__BoundingBoxes__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
yolo_msgs__msg__BoundingBoxes__Sequence__destroy(yolo_msgs__msg__BoundingBoxes__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    yolo_msgs__msg__BoundingBoxes__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
yolo_msgs__msg__BoundingBoxes__Sequence__are_equal(const yolo_msgs__msg__BoundingBoxes__Sequence * lhs, const yolo_msgs__msg__BoundingBoxes__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!yolo_msgs__msg__BoundingBoxes__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
yolo_msgs__msg__BoundingBoxes__Sequence__copy(
  const yolo_msgs__msg__BoundingBoxes__Sequence * input,
  yolo_msgs__msg__BoundingBoxes__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(yolo_msgs__msg__BoundingBoxes);
    yolo_msgs__msg__BoundingBoxes * data =
      (yolo_msgs__msg__BoundingBoxes *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!yolo_msgs__msg__BoundingBoxes__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          yolo_msgs__msg__BoundingBoxes__fini(&data[i]);
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
    if (!yolo_msgs__msg__BoundingBoxes__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
