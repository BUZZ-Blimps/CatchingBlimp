// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from yolo_msgs:msg/BoundingBox.idl
// generated code does not contain a copyright notice
#include "yolo_msgs/msg/detail/bounding_box__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
yolo_msgs__msg__BoundingBox__init(yolo_msgs__msg__BoundingBox * msg)
{
  if (!msg) {
    return false;
  }
  // probability
  // x_center
  // y_center
  // width
  // height
  // track_id
  // class_id
  return true;
}

void
yolo_msgs__msg__BoundingBox__fini(yolo_msgs__msg__BoundingBox * msg)
{
  if (!msg) {
    return;
  }
  // probability
  // x_center
  // y_center
  // width
  // height
  // track_id
  // class_id
}

bool
yolo_msgs__msg__BoundingBox__are_equal(const yolo_msgs__msg__BoundingBox * lhs, const yolo_msgs__msg__BoundingBox * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // probability
  if (lhs->probability != rhs->probability) {
    return false;
  }
  // x_center
  if (lhs->x_center != rhs->x_center) {
    return false;
  }
  // y_center
  if (lhs->y_center != rhs->y_center) {
    return false;
  }
  // width
  if (lhs->width != rhs->width) {
    return false;
  }
  // height
  if (lhs->height != rhs->height) {
    return false;
  }
  // track_id
  if (lhs->track_id != rhs->track_id) {
    return false;
  }
  // class_id
  if (lhs->class_id != rhs->class_id) {
    return false;
  }
  return true;
}

bool
yolo_msgs__msg__BoundingBox__copy(
  const yolo_msgs__msg__BoundingBox * input,
  yolo_msgs__msg__BoundingBox * output)
{
  if (!input || !output) {
    return false;
  }
  // probability
  output->probability = input->probability;
  // x_center
  output->x_center = input->x_center;
  // y_center
  output->y_center = input->y_center;
  // width
  output->width = input->width;
  // height
  output->height = input->height;
  // track_id
  output->track_id = input->track_id;
  // class_id
  output->class_id = input->class_id;
  return true;
}

yolo_msgs__msg__BoundingBox *
yolo_msgs__msg__BoundingBox__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo_msgs__msg__BoundingBox * msg = (yolo_msgs__msg__BoundingBox *)allocator.allocate(sizeof(yolo_msgs__msg__BoundingBox), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(yolo_msgs__msg__BoundingBox));
  bool success = yolo_msgs__msg__BoundingBox__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
yolo_msgs__msg__BoundingBox__destroy(yolo_msgs__msg__BoundingBox * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    yolo_msgs__msg__BoundingBox__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
yolo_msgs__msg__BoundingBox__Sequence__init(yolo_msgs__msg__BoundingBox__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo_msgs__msg__BoundingBox * data = NULL;

  if (size) {
    data = (yolo_msgs__msg__BoundingBox *)allocator.zero_allocate(size, sizeof(yolo_msgs__msg__BoundingBox), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = yolo_msgs__msg__BoundingBox__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        yolo_msgs__msg__BoundingBox__fini(&data[i - 1]);
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
yolo_msgs__msg__BoundingBox__Sequence__fini(yolo_msgs__msg__BoundingBox__Sequence * array)
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
      yolo_msgs__msg__BoundingBox__fini(&array->data[i]);
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

yolo_msgs__msg__BoundingBox__Sequence *
yolo_msgs__msg__BoundingBox__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  yolo_msgs__msg__BoundingBox__Sequence * array = (yolo_msgs__msg__BoundingBox__Sequence *)allocator.allocate(sizeof(yolo_msgs__msg__BoundingBox__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = yolo_msgs__msg__BoundingBox__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
yolo_msgs__msg__BoundingBox__Sequence__destroy(yolo_msgs__msg__BoundingBox__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    yolo_msgs__msg__BoundingBox__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
yolo_msgs__msg__BoundingBox__Sequence__are_equal(const yolo_msgs__msg__BoundingBox__Sequence * lhs, const yolo_msgs__msg__BoundingBox__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!yolo_msgs__msg__BoundingBox__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
yolo_msgs__msg__BoundingBox__Sequence__copy(
  const yolo_msgs__msg__BoundingBox__Sequence * input,
  yolo_msgs__msg__BoundingBox__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(yolo_msgs__msg__BoundingBox);
    yolo_msgs__msg__BoundingBox * data =
      (yolo_msgs__msg__BoundingBox *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!yolo_msgs__msg__BoundingBox__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          yolo_msgs__msg__BoundingBox__fini(&data[i]);
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
    if (!yolo_msgs__msg__BoundingBox__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
