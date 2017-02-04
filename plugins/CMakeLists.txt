cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo 8 REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(SASCPlugin SHARED SASCPlugin.cc)
target_link_libraries(SASCPlugin ${GAZEBO_LIBRARIES})

install(TARGETS SASCPlugin DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/plugins)
