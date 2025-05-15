""" utils.py offers different utilities

"""

import sys
import subprocess
import os
import gdown

import formatter
from formatter import Color
from globals import (
    SEPARATOR,
    ESMINI_CMAKE_TARGET_FLAGS,
    ESMINI_COMPILERS,
    ESMINI_DIRECTORY_EXTERNALS,
)


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


def get_os():
    """get_os gets the operating system type"""

    if sys.platform == "linux":
        return "linux"


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

    Inputs
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


def download_entity(source, name, data):
    """download_entity downloads entity from

    Inputs
    ----------
        name (str): name of the entity

        source (str): source to download

        data (dict): information about what to download
    """

    if source == "google-drive":
        print(
            "Downloading "
            + formatter.format_green(name + "_" + get_os())
            + " from "
            + formatter.format_green(source)
            + " ..."
        )
        os.system("mkdir -p " + data["destination"])
        gdown.download(
            data["src"],
            output=os.path.join(
                data["destination"], name + "_" + get_os() + data["extension"]
            ),
        )
