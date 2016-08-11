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
  "script/control_team_blue"
  "script/control_team_gold"
  "script/rqt_uctf"
  "script/spawn_blue"
  "script/spawn_gold"
  "script/spawn_one"
  DESTINATION bin
)
install(DIRECTORY
  "src/uctf"
  DESTINATION lib/python/site-packages
)
install(DIRECTORY
  "launch" "media" "models" "worlds"
  DESTINATION share/${PROJECT_NAME}
)

include(ExternalProject)

set(firmware_path "${CMAKE_CURRENT_SOURCE_DIR}/Firmware")
ExternalProject_Add(sitl_gazebo
  UPDATE_COMMAND "${firmware_path}/Tools/check_submodules.sh"
  SOURCE_DIR "${firmware_path}/Tools/sitl_gazebo"
  CMAKE_ARGS "-DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}"
  INSTALL_COMMAND ""
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/sitl_gazebo-prefix/src/sitl_gazebo-build/libmav_msgs.so"
  "${CMAKE_CURRENT_BINARY_DIR}/sitl_gazebo-prefix/src/sitl_gazebo-build/librotors_gazebo_motor_model.so"
  "${CMAKE_CURRENT_BINARY_DIR}/sitl_gazebo-prefix/src/sitl_gazebo-build/librotors_gazebo_imu_plugin.so"
  "${CMAKE_CURRENT_BINARY_DIR}/sitl_gazebo-prefix/src/sitl_gazebo-build/librotors_gazebo_mavlink_interface.so"
  "${CMAKE_CURRENT_BINARY_DIR}/sitl_gazebo-prefix/src/sitl_gazebo-build/librotors_gazebo_multirotor_base_plugin.so"
  DESTINATION lib
)

ExternalProject_Add(firmware
  UPDATE_COMMAND "${firmware_path}/Tools/check_submodules.sh"
  SOURCE_DIR "${firmware_path}"
  CMAKE_ARGS
    "-DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}"
    "-DCONFIG=posix_sitl_default"
  INSTALL_COMMAND ""
)

install(PROGRAMS
  "${CMAKE_CURRENT_BINARY_DIR}/firmware-prefix/src/firmware-build/src/firmware/posix/px4"
  DESTINATION bin
)
install(PROGRAMS
  "script/px4.sh"
  DESTINATION share/${PROJECT_NAME}
)
install(FILES
  "Firmware/ROMFS/px4fmu_common/mixers/quad_w.main.mix"
  "Firmware/ROMFS/sitl/mixers/plane_sitl.main.mix"
  DESTINATION share/${PROJECT_NAME}/mixers
)
install(FILES
  "Firmware/posix-configs/SITL/init/rcS_gazebo_iris"
  "Firmware/posix-configs/SITL/init/rcS_gazebo_plane"
  DESTINATION share/${PROJECT_NAME}/SITLinit
)

file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/dataman" "")
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/dataman"
  DESTINATION share/${PROJECT_NAME}/rootfs/fs/microsd
)
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/parameters" "")
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/parameters"
  DESTINATION share/${PROJECT_NAME}/rootfs/eeprom
)

install(FILES
  "plugin.xml"
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
