try:
    import pycodestyle
except ImportError:
    import pep8 as pycodestyle
import os


def test_pycodestyle():
    style = pycodestyle.StyleGuide()
    base_path = os.path.dirname(os.path.dirname(__file__))
    report = style.check_files([
        os.path.join(base_path, 'script', 'control_team_blue'),
        os.path.join(base_path, 'script', 'control_team_gold'),
        os.path.join(base_path, 'script', 'rqt_uctf'),
        os.path.join(base_path, 'script', 'spawn_blue'),
        os.path.join(base_path, 'script', 'spawn_gold'),
        os.path.join(base_path, 'src'),
        os.path.join(base_path, 'test'),
    ])
    assert not report.total_errors
