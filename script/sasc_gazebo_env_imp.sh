#!/usr/bin/env sh

export INSTALL_SPACE=/opt/sasc
. ${INSTALL_SPACE}/setup.sh
. ${INSTALL_SPACE}/share/gazebo-8/setup.sh
. ${INSTALL_SPACE}/share/uctf/setup.sh
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${INSTALL_SPACE}/share/gazebo_models
