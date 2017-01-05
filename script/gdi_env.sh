#!/usr/bin/env sh



export INSTALL_SPACE=/opt/sasc
. ${INSTALL_SPACE}/ugdi_venv/bin/activate
# cd ${INSTALL_SPACE}/ugdi_venv/ugdi && python run.py

exec "$@"
