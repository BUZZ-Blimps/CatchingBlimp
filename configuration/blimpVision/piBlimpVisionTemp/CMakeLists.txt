cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

project(cpp_opencv_test)

find_package(OpenCV REQUIRED)
find_package (Python COMPONENTS Interpreter Development)


message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

add_executable(piBlimpVision piBlimpVision.cpp)
target_link_libraries(piBlimpVision ${OpenCV_LIBS} wiringPi v4l1 v4l2)
