cmake_minimum_required(VERSION 3.5)
project(opencv_telemetry)

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
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)

include_directories(include)

# add the resized image message
set(msg_files
  "msg/ResizedImage.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES sensor_msgs
)

# add the subscriber example
add_executable(opencv_subscriber src/opencv_subscriber.cpp)
ament_target_dependencies(opencv_subscriber
  "cv_bridge"
  "image_transport"
  "OpenCV"
  "rclcpp")

# add the sync splitter
add_executable(split_sync_images src/split_sync_images.cpp)
ament_target_dependencies(split_sync_images
  "cv_bridge"
  "image_transport"
  "OpenCV"
  "rclcpp"
  "camera_info_manager")

# add the plugin example
add_library(resized_plugins src/manifest.cpp src/resized_publisher.cpp src/resized_subscriber.cpp)
ament_target_dependencies(resized_plugins
  "image_transport"
  "OpenCV"
  "cv_bridge"
  "pluginlib")

# add the publisher from video example
add_executable(minimal_opencv_ros2_node src/minimal_opencv_ros2_node.cpp)
ament_target_dependencies(minimal_opencv_ros2_node
  "cv_bridge"
  "image_transport"
  "OpenCV"
  "rclcpp")

# Install plugin descriptions
pluginlib_export_plugin_description_file(${PROJECT_NAME} resized_plugins.xml)

# Link interface
rosidl_target_interfaces(resized_plugins
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install executables
install(
  TARGETS opencv_subscriber resized_plugins minimal_opencv_ros2_node split_sync_images
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(cv_bridge image_transport pluginlib rosidl_default_runtime rclcpp sensor_msgs camera_info_manager)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
    calibration
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
