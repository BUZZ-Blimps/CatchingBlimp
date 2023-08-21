// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from yolo_msgs:msg/BoundingBox.idl
// generated code does not contain a copyright notice
#include "yolo_msgs/msg/detail/bounding_box__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
yolo_msgs__msg__BoundingBox__init(yolo_msgs__msg__BoundingBox * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    yolo_msgs__msg__BoundingBox__fini(msg);
    return false;
  }
  // x_center_balloon
  // y_center_balloon
  // width_balloon
  // height_balloon
  // x_center_y_goal
  // y_center_y_goal
  // width_y_goal
  // height_y_goal
  // x_center_o_goal
  // y_center_o_goal
  // width_o_goal
  // height_o_goal
  return true;
}

void
yolo_msgs__msg__BoundingBox__fini(yolo_msgs__msg__BoundingBox * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // x_center_balloon
  // y_center_balloon
  // width_balloon
  // height_balloon
  // x_center_y_goal
  // y_center_y_goal
  // width_y_goal
  // height_y_goal
  // x_center_o_goal
  // y_center_o_goal
  // width_o_goal
  // height_o_goal
}

bool
yolo_msgs__msg__BoundingBox__are_equal(const yolo_msgs__msg__BoundingBox * lhs, const yolo_msgs__msg__BoundingBox * rhs)
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
  // x_center_balloon
  if (lhs->x_center_balloon != rhs->x_center_balloon) {
    return false;
  }
  // y_center_balloon
  if (lhs->y_center_balloon != rhs->y_center_balloon) {
    return false;
  }
  // width_balloon
  if (lhs->width_balloon != rhs->width_balloon) {
    return false;
  }
  // height_balloon
  if (lhs->height_balloon != rhs->height_balloon) {
    return false;
  }
  // x_center_y_goal
  if (lhs->x_center_y_goal != rhs->x_center_y_goal) {
    return false;
  }
  // y_center_y_goal
  if (lhs->y_center_y_goal != rhs->y_center_y_goal) {
    return false;
  }
  // width_y_goal
  if (lhs->width_y_goal != rhs->width_y_goal) {
    return false;
  }
  // height_y_goal
  if (lhs->height_y_goal != rhs->height_y_goal) {
    return false;
  }
  // x_center_o_goal
  if (lhs->x_center_o_goal != rhs->x_center_o_goal) {
    return false;
  }
  // y_center_o_goal
  if (lhs->y_center_o_goal != rhs->y_center_o_goal) {
    return false;
  }
  // width_o_goal
  if (lhs->width_o_goal != rhs->width_o_goal) {
    return false;
  }
  // height_o_goal
  if (lhs->height_o_goal != rhs->height_o_goal) {
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
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // x_center_balloon
  output->x_center_balloon = input->x_center_balloon;
  // y_center_balloon
  output->y_center_balloon = input->y_center_balloon;
  // width_balloon
  output->width_balloon = input->width_balloon;
  // height_balloon
  output->height_balloon = input->height_balloon;
  // x_center_y_goal
  output->x_center_y_goal = input->x_center_y_goal;
  // y_center_y_goal
  output->y_center_y_goal = input->y_center_y_goal;
  // width_y_goal
  output->width_y_goal = input->width_y_goal;
  // height_y_goal
  output->height_y_goal = input->height_y_goal;
  // x_center_o_goal
  output->x_center_o_goal = input->x_center_o_goal;
  // y_center_o_goal
  output->y_center_o_goal = input->y_center_o_goal;
  // width_o_goal
  output->width_o_goal = input->width_o_goal;
  // height_o_goal
  output->height_o_goal = input->height_o_goal;
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
