""" utils.py offers different utilities

"""

import subprocess

from support.python.src.globals import (
    SEPARATOR,
    ESMINI_CMAKE_TARGET_FLAGS,
    ESMINI_COMPILERS,
)


class Color:  # pylint: disable=too-few-public-methods
    """color codes for pretty printing"""

    PURPLE = "\033[95m"
    CYAN = "\033[96m"
    DARKCYAN = "\033[36m"
    BLUE = "\033[94m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    RED = "\033[91m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"
    END = "\033[0m"


YES_OR_NO_STR = (
    Color.GREEN + " y(yes)" + Color.END + "/" + Color.RED + "n(no): " + Color.END
)


def subprocess_popen(commands, shell=False, cwd=None, env=None, return_code=False):
    """wrapper of subprocess.Popen for enabling testing

    Inputs
    --------
    commands (list) = commands to run in subprocess call

    shell (bool) = if commands should be executed in shell

    cwd = sets the current directory before subprocess call executes

    encoding (str) = which encoding to use

    env = defines the environment variables for the subprocess call

    return_code (bool) = if the return code should be returned or not

    Return
    --------
    (stdout,stderr) decoded to utf-8

    if return_code = True returns: (stdout,stderr,returncode)

    """

    with subprocess.Popen(
        commands,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        cwd=cwd,
        env=env,
        shell=shell,
    ) as process:
        stdout, stderr = process.communicate()
        if return_code:
            ret_code = process.returncode
    if return_code:
        return (stdout.decode("utf-8"), stderr.decode("utf-8"), ret_code)
    return (stdout.decode("utf-8"), stderr.decode("utf-8"))


def print_commands(args):
    """print arguments

    Inputs
    ----------
        args (dict): argument dictionary to print

    """

    print(
        "Selected " + Color.BOLD + Color.UNDERLINE + "command" + Color.END + " flags:"
    )
    for key, value in args.items():
        if isinstance(value, list):
            print("   " + Color.GREEN + key + ": " + Color.END + ", ".join(value))
        else:
            print("   " + Color.GREEN + key + ": " + Color.END + str(value))
    print("─" * SEPARATOR)


def yes_or_no(question):
    """yes_or_no asks user a yes or no question to decide an action

    Parameters
    ----------
        question (str): desired question statament to ask to user

    Return
    ------
        True or False
    """

    reply = str(input(question + YES_OR_NO_STR)).lower().strip()

    if (len(reply) == 1 and reply[0] == "y") or (
        len(reply) == 3 and reply[0:] == "yes"
    ):
        return True

    if (len(reply) == 1 and reply[0] == "n") or (len(reply) == 2 and reply[0:] == "no"):
        return False

    if not reply:
        return yes_or_no(Color.YELLOW + "You need to give me something " + Color.END)

    return yes_or_no("!!! Only available options are ->")


def get_available_cmake_target_flags():
    """get_available_cmake_target_flags"""

    build_str = ""
    build_str = (
        build_str
        + Color.BOLD
        + "\n"
        + "Available cmake target flag(s) for --target_flags key: "
        + Color.END
        + "\n"
    )
    for flag in ESMINI_CMAKE_TARGET_FLAGS:
        build_str = (
            build_str + Color.GREEN + "  " + flag + Color.END + " [true/false]" + "\n"
        )
    return build_str


def get_available_compilers():
    """get_available_compilers"""

    build_str = ""
    build_str = (
        build_str
        + Color.BOLD
        + "\n"
        + "Available compiler(s) for --compiler key: "
        + Color.END
        + "\n"
    )
    for compiler in ESMINI_COMPILERS:
        build_str = build_str + Color.GREEN + "  " + compiler + Color.END + "\n"
    return build_str
