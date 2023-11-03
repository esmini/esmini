# pylint: disable=duplicate-code
"""Run pylint"""
import os
import subprocess
import formatter

from globals import (
    SEPARATOR,
    ESMINI_DIRECTORY_SCRIPT,
)
from command.run.run import Run


import subprocess

def run_pylint():
    """
    Run pylint on a Python file with a specified configuration file.

    Args:
        file_path (str): Path to the Python file to lint.
        config_file (str): Path to the .pylintrc configuration file.
    """
    config_file=os.path.join(ESMINI_DIRECTORY_SCRIPT, ".pylintrc")
    files = Run.get_files_for_black_format()
    print("─" * SEPARATOR)
    for file in files:
        try:
            # Run pylint with the configuration file
            result = subprocess.run(
                ["pylint", f"--rcfile={config_file}", file],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,  # Decode stdout/stderr as strings
            )

            # Print the linting results
            if result.returncode == 0:
                print(
                    formatter.format_green("Linting passed: ")
                    + os.path.dirname(file)
                    + "/"
                    + formatter.format_green(os.path.basename(file))
                )
            else:
                print(
                    formatter.format_red("Linting failed:")
                    + os.path.dirname(file)
                    + "/"
                    + formatter.format_red(os.path.basename(file))
                )
                lines = result.stdout.splitlines()
                for line in lines:
                    if ".py:" in line:
                        parts = line.split(":")
                        filename_with_path = parts[0]
                        error_message = ":".join(parts[3:])
                        print(f"{filename_with_path}: {error_message}")
        except FileNotFoundError:
            print("pylint not found. Please ensure it is installed and in your PATH.")
    print("─" * SEPARATOR)
