#!/usr/bin/env sh

export INSTALL_SPACE=/opt/sasc
. ${INSTALL_SPACE}/bin/sasc_gazebo_env_imp.sh

exec "$@"
