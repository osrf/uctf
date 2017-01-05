#!/usr/bin/env sh
export INSTALL_SPACE=/opt/sasc
. ${INSTALL_SPACE}/venv3/bin/activate
export PYTHONPATH=${INSTALL_SPACE}/lib/python2.7/dist-packages:/opt/ros/kinetic/lib/python2.7/dist-packages:/usr/lib/python3/dist-packages
exec "$@"
