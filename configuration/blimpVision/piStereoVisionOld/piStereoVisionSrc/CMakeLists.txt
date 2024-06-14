cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

project(cpp_piStereoVision)
message(STATUS "Sanity Check")

find_package(OpenCV REQUIRED)
find_package (Python COMPONENTS Interpreter Development)

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

add_executable(piStereoVision piStereoVision.cpp serialib.cpp)
target_link_libraries(piStereoVision ${OpenCV_LIBS} v4l1 v4l2)
message(STATUS "Sanity Check 2")
