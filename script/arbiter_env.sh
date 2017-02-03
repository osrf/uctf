#!/usr/bin/env sh

unset PYTHONPATH
export INSTALL_SPACE=/opt/sasc
. ${INSTALL_SPACE}/venv3/bin/activate
exec "$@"
