#!/usr/bin/env sh
export INSTALL_SPACE=/opt/sasc
unset PYTHONPATH
. ${INSTALL_SPACE}/venv3/bin/activate
exec "$@"
