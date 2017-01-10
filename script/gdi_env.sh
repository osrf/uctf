#!/usr/bin/env sh

export INSTALL_SPACE=/opt/sasc
. ${INSTALL_SPACE}/ugdi_venv/bin/activate
PYTHONPATH=$PYTHONPATH:$INSTALL_SPACE/lib/python2.7/dist-packages/ap_lib

eval "$@"
