#!/usr/bin/env sh

. /opt/sasc/bin/sasc_gazebo_env_imp.sh
export ROS_HOSTNAME=192.168.2.1
export ROS_MASTER_URI=http://192.168.2.1:11311

eval "$@"
