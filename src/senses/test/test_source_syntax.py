"""Ensure every shipped Python source and launch file remains syntactically valid."""

import ast
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_python_sources_parse_without_importing_hardware_dependencies():
    paths = [PACKAGE_ROOT / 'setup.py']
    paths.extend((PACKAGE_ROOT / 'senses').glob('*.py'))
    paths.extend((PACKAGE_ROOT / 'launch').glob('*.launch.py'))

    failures = []
    for path in sorted(paths):
        try:
            ast.parse(path.read_text(), filename=str(path))
        except SyntaxError as error:
            failures.append(f'{path}: {error}')

    assert not failures, '\n'.join(failures)
