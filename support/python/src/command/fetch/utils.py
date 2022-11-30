# pylint: disable=duplicate-code
""" utils.py includes all shared entities between within fetch domain

"""

import os
import shutil

from support.python.src import formatter
from support.python.src.utils import yes_or_no
from support.python.src.globals import SEPARATOR, ESMINI_DIRECTORY_EXTERNAL


############################################################################################################################## # pylint: disable=line-too-long
######################################################### Fetch CLASS ######################################################## # pylint: disable=line-too-long
############################################################################################################################## # pylint: disable=line-too-long


def fetch_from_google_drive(  # pylint: disable=too-many-branches
    key, entity, entity_type, version=""
):
    """fetch_from_artifactory fetches the desired entities from artifactory

    Inputs
    ----------
        key (str): fetch or replace command

        entity (list): the list containing the data of the entity

        entity_type (str): the type of the entity that will be fetched from google drive

        version (str): version of the entity

    """

    root_dir = os.getcwd()
    if not os.path.exists(os.path.join(ESMINI_DIRECTORY_EXTERNAL, entity_type)):
        os.system("mkdir -p " + os.path.join(ESMINI_DIRECTORY_EXTERNAL, entity_type))
    os.chdir(os.path.join(ESMINI_DIRECTORY_EXTERNAL, entity_type))

    if key == "fetch":
        if os.path.exists(
            os.path.join(ESMINI_DIRECTORY_EXTERNAL, entity_type, entity[1])
        ):
            print(formatter.form("ALREADY EXISTS", entity[1]))
            if yes_or_no(formatter.format_yellow("WANT REPLACE ?", entity[1])):
                replace_from_google_drive(entity, entity_type, version)
            else:
                print("Allright :) \n")
                return
        else:
            # os.system(curl_key + entity[0] + "/" + version + "/" + entity[1] + ".zip")
            if os.system("unzip -q " + entity[1] + ".zip") == 0:
                print(formatter.format_green("FETCH", [entity[1], version]))
                os.system("rm " + entity[1] + ".zip")
            else:
                raise ValueError(formatter.format_red("FETCH", [entity[1], version]))
            print("─" * SEPARATOR)

    os.chdir(root_dir)


def replace_from_google_drive(entity, entity_type, version=""):
    """replace_from_google_drive replaces the desired dependency

    Inputs
    ----------
        key (str): fetch or replace command

        entity (list): the list containing the data of the entity

        entity_type (str): the type of the entity that will be fetched from google drive

        version (str): version of the entity

    """

    if os.path.exists(os.path.join(ESMINI_DIRECTORY_EXTERNAL, entity_type, entity[1])):
        shutil.rmtree(os.path.join(ESMINI_DIRECTORY_EXTERNAL, entity_type, entity[1]))
        print(formatter.format_green("REMOVAL", [entity[1]]))

    fetch_from_google_drive("fetch", entity, entity_type, version)
