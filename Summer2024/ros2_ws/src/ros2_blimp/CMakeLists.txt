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
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

set(SOURCE_FILES src/accelGCorrection.cpp src/BangBang.cpp src/baro_acc_kf.cpp src/OPI_IMU.cpp src/brushless.cpp src/EMAFilter.cpp src/Gimbal.cpp src/gyro_ekf.cpp src/Kalman_Filter_Tran_Vel_Est.cpp src/Madgwick_Filter.cpp src/MotorControl.cpp src/optical_ekf.cpp src/PID.cpp src/servo.cpp src/tripleBallGrabber.cpp src/blimp.cpp)

add_executable(blimp_auto ${SOURCE_FILES})
target_link_libraries(blimp_auto ${catkin_LIBRARIES} -lwiringPi)
target_link_libraries (blimp_auto Eigen3::Eigen)

ament_target_dependencies(blimp_auto rclcpp std_msgs geometry_msgs sensor_msgs)

install(TARGETS
  blimp_auto
  DESTINATION lib/${PROJECT_NAME})

ament_package()
