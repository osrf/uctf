import logging
import os
import sys

from pydocstyle import log
from pydocstyle import run_pydocstyle

# override default logging level which seems to be DEBUG
log.setLevel(logging.WARNING)


def empty(*args, **kwargs):
    pass


log.setLevel = empty


def test_pydocstyle():
    base_path = os.path.dirname(os.path.dirname(__file__))
    sys.argv = [
        'pydocstyle',
        os.path.join(base_path, 'script', 'control_team_blue'),
        os.path.join(base_path, 'script', 'control_team_gold'),
        os.path.join(base_path, 'script', 'rqt_uctf'),
        os.path.join(base_path, 'script', 'spawn_blue'),
        os.path.join(base_path, 'script', 'spawn_gold'),
        os.path.join(base_path, 'script', 'spawn_one'),
        os.path.join(base_path, 'src'),
        os.path.join(base_path, 'test'),
        '--add-ignore=D100,D101,D102,D103,D104',
    ]
    rc = run_pydocstyle()
    assert rc == 0
