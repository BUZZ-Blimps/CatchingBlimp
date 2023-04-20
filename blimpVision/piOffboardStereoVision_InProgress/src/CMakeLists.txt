cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

set(CMAKE_CXX_FLAGS "-Wno-psabi")

project(piOffboardStereoVision)

find_package(OpenCV REQUIRED)
find_package (Python COMPONENTS Interpreter Development)

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

add_executable(piOffboardStereoVision piOffboardStereoVision.cpp ComputerVision.cpp ComputerVisionOld.cpp PiComm.cpp serialib.cpp CameraHandler.cpp)
target_link_libraries(piOffboardStereoVision ${OpenCV_LIBS} pthread v4l1 v4l2)
