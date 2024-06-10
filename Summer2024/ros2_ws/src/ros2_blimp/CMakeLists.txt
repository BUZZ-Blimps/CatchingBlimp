cmake_minimum_required(VERSION 3.8)
project(ros2_blimp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(blimp_auto src/blimp.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

install(TARGETS
  blimp_auto
  DESTINATION lib/${PROJECT_NAME})

ament_package()
