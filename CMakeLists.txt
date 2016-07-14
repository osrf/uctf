cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

project(uctf)

install(FILES
  "package.xml"
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  "script/spawn"
  DESTINATION bin
)
install(DIRECTORY
  "src/uctf"
  DESTINATION lib/python/site-packages
)
install(DIRECTORY
  "launch" "models" "worlds"
  DESTINATION share/${PROJECT_NAME}
)

configure_file(
  cmake/setup.sh.in
  setup.sh
  @ONLY
)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/setup.sh"
  DESTINATION share/${PROJECT_NAME}
)
