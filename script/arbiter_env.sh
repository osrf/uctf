#!/usr/bin/env sh
export INSTALL_SPACE=/opt/sasc
. ${INSTALL_SPACE}/venv3/bin/activate
. ${INSTALL_SPACE}/setup.sh
exec "$@"
