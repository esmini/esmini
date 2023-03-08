# pylint: disable=duplicate-code
"""Run pytest and get code coverage measurement"""
import os
import sys

from globals import ESMINI_DIRECTORY_SUPPORT
from utils import subprocess_popen


def run_pytest() -> None:
    """Run pytest and measure coverage"""
    pytest_args = [
        "python3.8",
        "-m",
        "pytest",
        "-v",
        "--cov=src",
        "--cov-report",
        "xml",
        "--cov-report",
        "term",
        "test",
    ]
    stdout, stderr, return_code = subprocess_popen(
        pytest_args,
        cwd=os.path.join(ESMINI_DIRECTORY_SUPPORT, "python"),
        return_code=True,
    )
    if stdout:
        print(stdout)
    if stderr:
        print(stderr)

    sys.exit(return_code)
