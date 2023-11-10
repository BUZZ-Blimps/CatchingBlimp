cmake_minimum_required(VERSION 3.5)
project(stereo_combined)

# cmake_policy(SET CMP0121 OLD)

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
find_package(stereo_msgs REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_geometry REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_proc REQUIRED)
find_package(yolo_msgs REQUIRED)

add_executable(
    ${PROJECT_NAME}_node 
    src/stereo_combined_node.cpp
    src/StereoCombined.cpp
    src/edge_aware.cpp
    src/stereo_processor.cpp
    include/StereoCombined.hpp
    include/edge_aware.hpp
    include/stereo_processor.hpp
)
ament_target_dependencies(${PROJECT_NAME}_node
    rclcpp 
    std_msgs 
    sensor_msgs
    stereo_msgs
    cv_bridge
    image_transport
    image_geometry
    OpenCV
    camera_info_manager
    image_proc
    yolo_msgs
)

target_include_directories(
    ${PROJECT_NAME}_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(
    TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
    calibration
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
