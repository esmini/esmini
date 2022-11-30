# pylint: disable=duplicate-code
"""Run pylint"""
import os
import sys
from support.python.src.utils import subprocess_popen

from support.python.src.globals import ESMINI_DIRECTORY_ROOT


def run_pylint() -> None:
    """Run pylint"""
    pylint_args = [
        "python3.8",
        "-m",
        "pylint",
        "--output-format=parseable",
        "--output=pylint_report.txt",
        "--ignore-imports=yes",
        "support/python/src",
    ]

    stdout, stderr, return_code = subprocess_popen(
        pylint_args, cwd=os.path.join(ESMINI_DIRECTORY_ROOT), return_code=True
    )
    if stdout:
        print(stdout)
    if stderr:
        print(stderr)

    sys.exit(return_code)
