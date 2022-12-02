# pylint: disable=duplicate-code
""" utils.py includes all shared entities between within fetch domain

"""

import os
import shutil

from support.python.src import formatter
from support.python.src.utils import (
    yes_or_no,
    get_os,
    download_entity,
    subprocess_popen,
)
from support.python.src.globals import SEPARATOR, ESMINI_DIRECTORY_EXTERNALS


############################################################################################################################## # pylint: disable=line-too-long
######################################################### Fetch CLASS ######################################################## # pylint: disable=line-too-long
############################################################################################################################## # pylint: disable=line-too-long


def fetch_from_given_source(  # pylint: disable=too-many-branches
    key, source, entity_name, entity_data, version=""
):
    """fetch_from_given_source fetches the desired entities from artifactory

    Inputs
    ----------
        key (str): fetch or replace command

        source (str): the source to fetch data from

        entity_name (list): the name of the entity to be fetched

        entity_data (list): the data of the entity to be fetched

        version (str): version of the entity

    """

    if key == "fetch":
        if os.path.exists(os.path.join(ESMINI_DIRECTORY_EXTERNALS, entity_name)):
            print(formatter.format_yellow("ALREADY EXISTS " + entity_name))
            if yes_or_no(formatter.format_yellow("WANT REPLACE ? " + entity_name)):
                replace_from_given_source(source, entity_name, entity_data, version)
            else:
                print("Allright :) \n")
                return
        else:
            download_entity(source, entity_name, entity_data)
            if entity_data["extension"] == ".7z":
                cmd = [
                    "7z",
                    "x",
                    os.path.join(
                        entity_data["destination"],
                        entity_name + "_" + get_os() + entity_data["extension"],
                    ),
                    "-oc:" + entity_data["destination"],
                ]
                stdout, stderr, return_code = subprocess_popen(
                    cmd, cwd=os.path.join(ESMINI_DIRECTORY_EXTERNALS), return_code=True
                )

                if return_code != 0:
                    print(stdout)
                    print(stderr)
                    raise ValueError(formatter.format_red("FAIL"))

                print(
                    formatter.format_green("SUCCESS ")
                    + "-> "
                    + formatter.format_green("FETCH ")
                    + "-> "
                    + formatter.format_green(entity_name + "_" + get_os())
                    + " "
                    + formatter.format_green(version)
                )
                os.system(
                    "rm "
                    + os.path.join(
                        entity_data["destination"],
                        entity_name + "_" + get_os() + entity_data["extension"],
                    )
                )
            else:
                raise ValueError(formatter.format_red("FAIL"))
            print("─" * SEPARATOR)


def replace_from_given_source(source, entity_name, entity_data, version=""):
    """replace_from_given_source replaces the desired dependency

    Inputs
    ----------
        source (str): the source to fetch data from

        entity (list): the list containing the data of the entity

        entity_type (str): the type of the entity that will be fetched from google drive

        version (str): version of the entity

    """

    if os.path.exists(entity_data["destination"]):
        shutil.rmtree(os.path.join(entity_data["destination"]))
        print(formatter.format_green("REMOVAL " + entity_name))

    fetch_from_given_source("fetch", source, entity_name, entity_data, version)
