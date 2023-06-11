cmake_minimum_required(VERSION 3.5)
project(blimp_telemetry)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(blimp_telemetry_node src/blimp_telemetry_node.cpp)
ament_target_dependencies(
   blimp_telemetry_node
   rclcpp 
   std_msgs
   sensor_msgs 
   tf2 
   tf2_ros 
   geometry_msgs)


install(
  TARGETS
  blimp_telemetry_node
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
