""" utils.py includes all shared entities between within generate domain

"""

import os

from support.python.src import formatter
from support.python.src.utils import print_commands

############################################################################################################################## # pylint: disable=line-too-long
######################################################### Generate CLASS ##################################################### # pylint: disable=line-too-long
############################################################################################################################## # pylint: disable=line-too-long


def utils_resolve_args(args):
    """resolves the arguments in case there is conflicted request"""
    print_commands(args)


def utils_get_all_external(externalpath):
    """
    utils_get_all_external

    Inputs
    --------
    externalpath str: path to the external directory to fetch

    Returns
    ---------
    list containing name of all directory in external

    """

    externals = os.listdir(externalpath)
    if len(externals) == 0:
        raise ValueError(
            formatter.get_error_message(
                "NO FOLDER FOUND UNDER",
                externalpath,
            )
        )
    return externals
