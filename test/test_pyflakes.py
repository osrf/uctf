import os

from pyflakes import reporter
from pyflakes.api import checkRecursive


def test_pyflakes():
    rep = reporter._makeDefaultReporter()
    base_path = os.path.dirname(os.path.dirname(__file__))
    paths = [
        os.path.join(base_path, 'src'),
        os.path.join(base_path, 'test'),
    ]
    warnings = checkRecursive(paths, rep)
    assert warnings == 0
