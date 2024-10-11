# esmini - Environment Simulator Minimalistic
# https://github.com/esmini/esmini
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
#
# Copyright (c) partners of Simulation Scenarios
# https://sites.google.com/view/simulationscenarios


import xmlschema
from lxml import etree
import argparse
import os
import sys
from pathlib import Path

SCHEMA_MAPPINGS = {
    "xosc": {
        "0": "OpenSCENARIOv1.0.xsd",
        "1": "OpenSCENARIOv1.1.1.xsd",
        "2": "OpenSCENARIOv1.2.xsd",
        "3": "OpenSCENARIOv1.3.xsd",
    },
    "xodr": {
        "4": "OpenDRIVE_1.4H.xsd",
        "5": "OpenDRIVE_1.5.xsd",
        "6": "OpenDRIVE_1.6/opendrive_16_core.xsd",
        "7": "OpenDRIVE_1.7/localSchema/opendrive_17_core.xsd",
        "8": "OpenDRIVE_1.8/local_schema/OpenDRIVE_Core.xsd",
    },
}


class XmlValidation:
    def __init__(self):
        self.use_xsd11 = False
        self.xml_file_names = []
        self.xsd_files_path = r"resources/schema/"
        self.errors = []
        self.count_of_files_validated = 0
        self.count_of_files_failed = 0

    def is_xml11_needed(self):
        return self.use_xsd11

    def get_count_of_files_validated(self):
        return self.count_of_files_validated

    def set_xml11_needed(self):
        self.use_xsd11 = True

    def get_xml_files_to_validate(self):
        return self.xml_file_names

    def set_xml_files(self, file_name):
        self.xml_file_names.append(file_name)

    def get_xsd_files_path(self):
        return self.xsd_files_path

    def get_xml_type(self, file_path):
        return "xodr" if file_path.endswith(".xodr") else "xosc"

    def print_errors(self):
        if self.count_of_files_failed > 0:
            counter = 0
            for item in self.errors:
                if counter == 0:
                    print(f"Files failed to validate are:")
                print(item)
                counter += 1
            raise ValueError(
                f"Validate scheme failed. {self.count_of_files_failed} files failed to validate. Check the log."
            )
        else:
            print(f"{self.get_count_of_files_validated()} Files validated")

    def get_xml_header_minor_revision(self, file_path):
        try:
            tree = etree.parse(file_path)
            root = tree.getroot()
            if self.get_xml_type(file_path) == "xosc":
                header = root.findall("./FileHeader")
            else:
                header = root.findall("./header")
            revMinor = header[0].attrib["revMinor"]
            if self.get_xml_type(file_path) == "xodr" and revMinor == "8":
                self.set_xml11_needed()
            return revMinor
        except:
            raise ValueError(f"XML Parsing Error found in {file_path}. Check log")

    def get_xsd_to_validate(self, revMinor, type_):
        if type_ in SCHEMA_MAPPINGS and revMinor in SCHEMA_MAPPINGS[type_]:
            return os.path.join(self.xsd_files_path, SCHEMA_MAPPINGS[type_][revMinor])
        else:
            return None

    def validate(self, xml_file, schema_file):
        if schema_file is None:
            raise ValueError(
                f"Unknown header found in file {xml_file}. Check revisions"
            )
        try:
            # Create the XMLSchema object once outside the loop
            my_schema = (
                xmlschema.XMLSchema11(schema_file)
                if self.is_xml11_needed()
                else xmlschema.XMLSchema(schema_file)
            )

            # Parse the XML document only once
            parser = etree.XMLParser(recover=False)
            document_tree = etree.parse(xml_file, parser)
            errors = list(my_schema.iter_errors(document_tree))

            if not len(errors) == 0:
                print(f"{xml_file} \033[31m fails to validates. \033[0m")
                self.count_of_files_failed += 1
                # Iterate over errors and print them directly, avoiding an intermediate list
                for error in errors:
                    elem = error.elem
                    line_number = elem.sourceline if elem is not None else "unknown"
                    element_name = elem.tag if elem is not None else "unknown"
                    self.errors.append(
                        f"{xml_file}:{line_number}: Schemas validity error : element '{element_name}' : {error.reason}"
                    )
            else:
                print(f"{xml_file} \033[32m validates.\033[0m")
                self.count_of_files_validated += 1

        except xmlschema.validators.exceptions.XMLSchemaValidationError as e:
            raise ValueError(f"An error occurred during validation: {e}")

    def convert_arguments(self, args):
        if len(args) == 1 and "\n" in args[0]:
            # remove /n
            return args[0].split("\n")
        else:
            return args

    def validate_argument(self, paths):
        for path in paths:
            path = Path(path)
            # Check if it's a valid file or directory
            if path.is_file():
                # Validate the file extension
                if path.suffix in [".xosc", ".xodr"]:
                    return
                else:
                    raise argparse.ArgumentTypeError(
                        f"File '{path}' has an unsupported extension. Only .xosc and .xodr are allowed."
                    )
            elif path.is_dir():
                return
            else:
                raise argparse.ArgumentTypeError(
                    f"'{path}' is neither a valid file nor a directory."
                )

    def set_xml_files_to_validate(self, file_paths):
        for path in file_paths:
            path = Path(path)
            if os.path.isfile(path):
                file_path = os.path.join(os.getcwd() + "/" + str(path))
                self.set_xml_files(file_path)
            # Check if the path is a directory and walk through its files
            elif os.path.isdir(path):
                for root, dirs, files in os.walk(path):
                    for file in files:
                        # Only consider .xosc or .xodr files
                        if file.endswith((".xosc", ".xodr")):
                            file_path = os.path.join(root, file)
                            self.set_xml_files(file_path)

    def main(self, arg):
        paths = self.convert_arguments(arg)
        self.validate_argument(paths)
        self.set_xml_files_to_validate(paths)
        for file in self.get_xml_files_to_validate():
            revMinor = self.get_xml_header_minor_revision(file)
            self.validate(
                file, self.get_xsd_to_validate(revMinor, self.get_xml_type(file))
            )


if __name__ == "__main__":
    validator = XmlValidation()
    validator.main(sys.argv[1:])
    validator.print_errors()
