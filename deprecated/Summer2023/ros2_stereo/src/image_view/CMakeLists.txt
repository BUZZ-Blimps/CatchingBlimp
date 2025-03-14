cmake_minimum_required(VERSION 3.5)
project(image_view)

# cmake_policy(SET CMP0121 OLD)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Boost REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(camera_calibration_parsers REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(stereo_msgs REQUIRED)

include_directories(include)

# Deal with the GUI's
if(ANDROID)
  return()
endif()



ament_auto_add_library(${PROJECT_NAME}_nodes SHARED
  src/disparity_view_node.cpp
  src/extract_images_node.cpp
  src/image_view_node.cpp
  src/image_saver_node.cpp
  src/stereo_view_node.cpp
  src/video_recorder_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}_nodes
  camera_calibration_parsers
  cv_bridge
  image_transport
  message_filters
  rclcpp
  rclcpp_components
  sensor_msgs
  std_srvs
  stereo_msgs
)

target_link_libraries(${PROJECT_NAME}_nodes
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_compile_definitions(${PROJECT_NAME}_nodes
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
rclcpp_components_register_nodes(${PROJECT_NAME}_nodes
  "${PROJECT_NAME}::DisparityViewNode"
  "${PROJECT_NAME}::ExtractImagesNode"
  "${PROJECT_NAME}::ImageViewNode"
  "${PROJECT_NAME}::ImageSaverNode"
  "${PROJECT_NAME}::StereoViewNode"
  "${PROJECT_NAME}::VideoRecorderNode"
)

# Image viewers
ament_auto_add_executable(disparity_view
  src/disparity_view.cpp
)
target_link_libraries(disparity_view
  ${PROJECT_NAME}_nodes
)
add_dependencies(disparity_view
  ${PROJECT_NAME}_nodes
)
ament_target_dependencies(disparity_view
  "OpenCV"
)


ament_auto_add_executable(${PROJECT_NAME}
  src/image_view.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_nodes
)
add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_nodes
)

ament_auto_add_executable(stereo_view
  src/stereo_view.cpp
)
target_link_libraries(stereo_view
  ${PROJECT_NAME}_nodes
)
add_dependencies(stereo_view
  ${PROJECT_NAME}_nodes
)

# Other Tools
ament_auto_add_executable(extract_images
  src/extract_images.cpp
)
target_link_libraries(extract_images
  ${PROJECT_NAME}_nodes
)
add_dependencies(extract_images
  ${PROJECT_NAME}_nodes
)

ament_auto_add_executable(image_saver
  src/image_saver.cpp
)
target_link_libraries(image_saver
  ${PROJECT_NAME}_nodes
)
add_dependencies(image_saver
  ${PROJECT_NAME}_nodes
)

ament_auto_add_executable(video_recorder
  src/video_recorder.cpp
)
target_link_libraries(video_recorder
  ${PROJECT_NAME}_nodes
)
add_dependencies(video_recorder
  ${PROJECT_NAME}_nodes
)

if(BUILD_TESTING)
  find_package(ament_lint_auto)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
