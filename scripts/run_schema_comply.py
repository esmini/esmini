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
        self.xml_file_names = []
        self.xsd_files_path = r"resources/schema/"
        self.errors = []
        self.count_of_files_validated = 0
        self.count_of_files_failed = 0

    def get_count_of_files_validated(self):
        return self.count_of_files_validated

    def get_xml_files_to_validate(self):
        return self.xml_file_names

    def set_xml_files(self, file_name):
        self.xml_file_names.append(file_name)

    def get_xsd_files_path(self):
        return self.xsd_files_path

    def get_xml_type(self, file_path):
        return "xodr" if file_path.endswith(".xodr") else "xosc"

    def increase_files_failed_to_validate(self):
        self.count_of_files_failed += 1

    def get_count_of_files_failed_to_validate(self):
        return self.count_of_files_failed

    def log_error(self, error_msg, file):
        print(f"[NOT OK] {file}", flush=True)
        self.errors.append(error_msg)
        self.increase_files_failed_to_validate()

    def print_errors(self):
        if self.get_count_of_files_failed_to_validate() > 0:
            print(f"Issues:")
            for item in self.errors:
                print(item)
            print(
                "{} files validated {} failed".format(
                    self.get_count_of_files_validated(),
                    self.get_count_of_files_failed_to_validate(),
                )
            )
        else:
            print(f"{self.get_count_of_files_validated()} files validated")

    def get_xml_header_minor_revision(self, file_path):
        if self.get_xml_type(file_path) == "xosc":
            header = self.root.findall("./FileHeader")
        else:
            header = self.root.findall("./header")
        if (
            len(header) > 0 and "revMinor" in header[0].attrib
        ):  # check header and revMinor attribute available
            revMinor = header[0].attrib["revMinor"]
        else:
            error_msg = "{}: Check header, Either header or revMinor missing ".format(
                file_path
            )
            self.log_error(error_msg, file_path)
            return None
        return revMinor

    def get_xsd_to_validate(self, revMinor, file):
        type_ = self.get_xml_type(file)
        if type_ in SCHEMA_MAPPINGS and revMinor in SCHEMA_MAPPINGS[type_]:
            return os.path.join(self.xsd_files_path, SCHEMA_MAPPINGS[type_][revMinor])
        else:
            error_msg = "{}: minor version {} found. Supported: {}".format(
                file,
                revMinor,
                ", ".join(str(key) for key in SCHEMA_MAPPINGS[type_].keys()),
            )
            self.log_error(error_msg, file)
            return None

    def get_scheme(self, xml_file, schema_file):
        if schema_file is not None:
            try:
                tmp_tree = etree.parse(schema_file)
            except etree.ParseError as e:
                error_msg = "{}: Schema {} parsing error: {}".format(
                    xml_file, os.path.basename(schema_file), e.msg
                )
                self.log_error(error_msg, xml_file)
                return None

            try:
                self.xml_version = tmp_tree.docinfo.xml_version

                if self.xml_version == "1.0":
                    my_schema = xmlschema.XMLSchema(schema_file)
                elif self.xml_version == "1.1":
                    my_schema = xmlschema.XMLSchema11(schema_file)
                else:
                    error_msg = "{}: Unsupported XML version: {}".format(
                        xml_file, self.xml_version
                    )
                    self.log_error(error_msg, xml_file)
                    return None
            except xmlschema.validators.exceptions.XMLSchemaParseError as e:
                error_msg = "{}: Schema error {}. Update xml version to 1.1?".format(
                    xml_file, e.message
                )
                self.log_error(error_msg, xml_file)
                return None
            return my_schema

    def validate(self, xml_file, schema_file):
        my_schema = self.get_scheme(xml_file, schema_file)
        if my_schema is not None:
            parser = etree.XMLParser(recover=False)
            document_tree = etree.parse(xml_file, parser)
            errors = list(my_schema.iter_errors(document_tree))

            if not len(errors) == 0:
                print(f"[NOT OK] {xml_file}", flush=True)
                self.increase_files_failed_to_validate()
                # Iterate over errors and print them directly, avoiding an intermediate list
                for error in errors:
                    elem = error.elem
                    line_number = elem.sourceline if elem is not None else "unknown"
                    element_name = elem.tag if elem is not None else "unknown"
                    self.errors.append(
                        f"{xml_file}:{line_number}: '{element_name}' : {error.reason}"
                    )
            else:
                print(f"[OK] {xml_file}", flush=True)
                self.count_of_files_validated += 1

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
                self.set_xml_files(str(path))
            # Check if the path is a directory and walk through its files
            elif os.path.isdir(path):
                for root, dirs, files in os.walk(path):
                    for file in files:
                        # Only consider .xosc or .xodr files
                        if file.endswith((".xosc", ".xodr")):
                            file_path = os.path.join(root, file)
                            self.set_xml_files(file_path)

    def open_file(self, file):
        try:
            self.tree = etree.parse(file)
            self.root = self.tree.getroot()
            return True
        except etree.ParseError as e:
            self.log_error("{}: Parsing error: {}".format(file, e.msg), file)
            return False

    def main(self, arg):
        paths = self.convert_arguments(arg)
        self.validate_argument(paths)
        self.set_xml_files_to_validate(paths)
        for file in self.get_xml_files_to_validate():
            if self.open_file(file):
                revMinor = self.get_xml_header_minor_revision(file)
                if revMinor is not None:
                    self.validate(file, self.get_xsd_to_validate(revMinor, file))


if __name__ == "__main__":

    if len(sys.argv) < 2:
        print(
            "Usage: python run_schema_comply.py <path1> [path2] [path3] ...\n\n   path can be file or directory"
        )
        exit(-1)

    validator = XmlValidation()
    validator.main(sys.argv[1:])
    validator.print_errors()

    if (
        validator.get_count_of_files_validated() > 0
        and validator.get_count_of_files_failed_to_validate() == 0
    ):
        exit(0)  # at least one file has been validated and no failures
    else:
        exit(1)  # exit code 1 to indicate generic error from script