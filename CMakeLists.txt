cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

project(uctf)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

find_package(roscpp REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})

set(PROJECT_LIB_ROBOT_PLUGIN_NAME "RobotPlugin")

include_directories(include)

add_subdirectory(src)
