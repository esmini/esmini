""" main.py is the major script to handle everything about this repo

"""

import argparse
import sys

from command.run import pylint
from command.run import pytest
from cli.cli import CLI
from command.fetch.dependency import Dependency
from command.generate.opendrive import OpenDrive

##############################################################################################################################
############################################################ MAIN ############################################################
##############################################################################################################################


class Start(object):
    def __init__(self):
        pass

    def execute(self, cli_argv):
        cli = CLI()
        command_list = cli.parse(cli_argv)
        if "fetch" == command_list[1].name:
            if "dependency" == command_list[2].name:
                Dependency.fetch_dependency(
                    command_list[1].name, command_list[2].parsed_args
                )
        elif "replace" == command_list[1].name:
            if "dependency" == command_list[2].name:
                Dependency.replace_dependency(command_list[2].parsed_args)
        elif "generate" == command_list[1].name:
            if "openscenario" == command_list[2].name:
                pass
            elif "opendrive" == command_list[2].name:
                opendrive = OpenDrive()
                opendrive.generate_opendrive("1.7")

    def execution_scheduler(self, cli_argv_list):
        for cli_argv in cli_argv_list:
            self.execute(cli_argv)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(add_help=False)
    unknown_args, command_args = parser.parse_known_args()
    command_args.insert(0, sys.argv[0])

    start = Start()
    start.execute(command_args)
