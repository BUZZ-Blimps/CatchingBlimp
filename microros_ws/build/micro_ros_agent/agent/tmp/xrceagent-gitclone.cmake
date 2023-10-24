# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

if(EXISTS "/home/corelab/GitHub/CatchingBlimp/microros_ws/build/micro_ros_agent/agent/src/xrceagent-stamp/xrceagent-gitclone-lastrun.txt" AND EXISTS "/home/corelab/GitHub/CatchingBlimp/microros_ws/build/micro_ros_agent/agent/src/xrceagent-stamp/xrceagent-gitinfo.txt" AND
  "/home/corelab/GitHub/CatchingBlimp/microros_ws/build/micro_ros_agent/agent/src/xrceagent-stamp/xrceagent-gitclone-lastrun.txt" IS_NEWER_THAN "/home/corelab/GitHub/CatchingBlimp/microros_ws/build/micro_ros_agent/agent/src/xrceagent-stamp/xrceagent-gitinfo.txt")
  message(STATUS
    "Avoiding repeated git clone, stamp file is up to date: "
    "'/home/corelab/GitHub/CatchingBlimp/microros_ws/build/micro_ros_agent/agent/src/xrceagent-stamp/xrceagent-gitclone-lastrun.txt'"
  )
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E rm -rf "/home/corelab/GitHub/CatchingBlimp/microros_ws/build/micro_ros_agent/agent/src/xrceagent"
  RESULT_VARIABLE error_code
)
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: '/home/corelab/GitHub/CatchingBlimp/microros_ws/build/micro_ros_agent/agent/src/xrceagent'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "/usr/bin/git" 
            clone --no-checkout --config "advice.detachedHead=false" "https://github.com/eProsima/Micro-XRCE-DDS-Agent.git" "xrceagent"
    WORKING_DIRECTORY "/home/corelab/GitHub/CatchingBlimp/microros_ws/build/micro_ros_agent/agent/src"
    RESULT_VARIABLE error_code
  )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once: ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/eProsima/Micro-XRCE-DDS-Agent.git'")
endif()

execute_process(
  COMMAND "/usr/bin/git" 
          checkout "ros2" --
  WORKING_DIRECTORY "/home/corelab/GitHub/CatchingBlimp/microros_ws/build/micro_ros_agent/agent/src/xrceagent"
  RESULT_VARIABLE error_code
)
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: 'ros2'")
endif()

set(init_submodules TRUE)
if(init_submodules)
  execute_process(
    COMMAND "/usr/bin/git" 
            submodule update --recursive --init 
    WORKING_DIRECTORY "/home/corelab/GitHub/CatchingBlimp/microros_ws/build/micro_ros_agent/agent/src/xrceagent"
    RESULT_VARIABLE error_code
  )
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: '/home/corelab/GitHub/CatchingBlimp/microros_ws/build/micro_ros_agent/agent/src/xrceagent'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy "/home/corelab/GitHub/CatchingBlimp/microros_ws/build/micro_ros_agent/agent/src/xrceagent-stamp/xrceagent-gitinfo.txt" "/home/corelab/GitHub/CatchingBlimp/microros_ws/build/micro_ros_agent/agent/src/xrceagent-stamp/xrceagent-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
)
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: '/home/corelab/GitHub/CatchingBlimp/microros_ws/build/micro_ros_agent/agent/src/xrceagent-stamp/xrceagent-gitclone-lastrun.txt'")
endif()
