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
from pathlib import Path
from itertools import islice
from urllib.parse import urlsplit
from urllib.request import url2pathname

SCHEMA_MAPPINGS = {
    "xosc": {
        "0": "OpenSCENARIOv1.0.xsd",
        "1": "OpenSCENARIOv1.1.1.xsd",
        "2": "OpenSCENARIOv1.2.xsd",
        "3": "OpenSCENARIOv1.3.xsd",
        "4": "OpenSCENARIOv1.4.xsd",
    },
    "xodr": {
        "4": "OpenDRIVE_1.4H.xsd",
        "5": "OpenDRIVE_1.5.xsd",
        "6": "OpenDRIVE_1.6/opendrive_16_core.xsd",
        "7": "OpenDRIVE_1.7/localSchema/opendrive_17_core.xsd",
        "8": "OpenDRIVE_1.8/local_schema/OpenDRIVE_Core.xsd",
        "9": "OpenDRIVE_1.9/OpenDRIVE_Core.xsd",
    },
}


class XmlValidation:
    def __init__(self, silent=False, fail_fast=False):
        self.silent = silent
        self.fail_fast = fail_fast
        self.xml_file_names = []
        # default location of schema files is under the resources folder, next to the script folder
        self.xsd_files_path = os.path.join(os.path.dirname(__file__), "../resources/schema")
        self.errors = []
        self.count_of_files_validated = 0
        self.count_of_files_failed = 0

    def get_count_of_files_validated(self):
        return self.count_of_files_validated

    def get_xml_files_to_validate(self):
        return self.xml_file_names

    def set_xml_files(self, file_name):
        self.xml_file_names.append(file_name)

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

    def get_referenced_schema(self, xml_file):
        location = self.root.get(
            "{http://www.w3.org/2001/XMLSchema-instance}noNamespaceSchemaLocation"
        )
        if location is None:
            return None
        location = location.strip()
        if not location:
            self.log_error(f"{xml_file}: Empty noNamespaceSchemaLocation", xml_file)
            return None
        uri = urlsplit(location)
        if uri.scheme.lower() == "file":
            if uri.netloc not in ("", "localhost") or uri.query or uri.fragment:
                self.log_error(f"{xml_file}: Unsupported schema URI {location}", xml_file)
                return None
            schema = Path(url2pathname(uri.path))
        elif uri.scheme and not Path(location).is_absolute():
            self.log_error(
                f"{xml_file}: Only local schema paths and file URIs are supported: {location}",
                xml_file,
            )
            return None
        else:
            schema = Path(location)
        if not schema.is_absolute():
            schema = Path(xml_file).resolve().parent / schema
        if not schema.is_file():
            if not self.silent:
                print(
                    f"[WARNING] {xml_file}: Referenced schema not found: {schema}. "
                    "Falling back to the version-based schema.",
                    flush=True,
                )
            return None
        return str(schema.resolve())

    def get_scheme(self, xml_file, schema_file):
        if schema_file is not None:
            try:
                tmp_tree = etree.parse(schema_file)
            except (etree.ParseError, OSError) as e:
                error_msg = "{}: Schema {} parsing error: {}".format(
                    xml_file, os.path.basename(schema_file), str(e)
                )
                self.log_error(error_msg, xml_file)
                return None

            try:
                root = tmp_tree.getroot()
                schema_min_version = root.attrib.get(
                    "{http://www.w3.org/2007/XMLSchema-versioning}minVersion"
                )

                # Legacy OpenDRIVE 1.8 schemas signal XSD 1.1 through the XML declaration.
                if schema_min_version == "1.1" or tmp_tree.docinfo.xml_version == "1.1":
                    my_schema = xmlschema.XMLSchema11(schema_file)
                else:
                    my_schema = xmlschema.XMLSchema(schema_file)
            except xmlschema.validators.exceptions.XMLSchemaParseError as e:
                error_msg = "{}: Schema error {}".format(
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
            validation_errors = my_schema.iter_errors(document_tree)
            errors = list(islice(validation_errors, 1) if self.fail_fast else validation_errors)

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
            if self.fail_fast and self.count_of_files_failed > 0:
                break
            if self.open_file(file):
                failures_before = self.count_of_files_failed
                if self.root.get(
                    "{http://www.w3.org/2001/XMLSchema-instance}noNamespaceSchemaLocation"
                ) is not None:
                    schema = self.get_referenced_schema(file)
                    if schema is not None:
                        self.validate(file, schema)
                        continue
                    if self.count_of_files_failed != failures_before:
                        continue
                revMinor = self.get_xml_header_minor_revision(file)
                if revMinor is not None:
                    self.validate(file, self.get_xsd_to_validate(revMinor, file))


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Validate OpenSCENARIO and OpenDRIVE XML")
    parser.add_argument(
        "--silent",
        action="store_true",
        help="suppress missing-schema fallback warnings; validation results and errors remain visible",
    )
    parser.add_argument("paths", nargs="+", help="files or directories to validate")
    parser.add_argument(
        "--fail-fast",
        action="store_true",
        help="stop at the first validation error, without checking remaining files",
    )
    args = parser.parse_args()
    validator = XmlValidation(silent=args.silent, fail_fast=args.fail_fast)
    validator.main(args.paths)
    validator.print_errors()

    if (
        validator.get_count_of_files_validated() > 0
        and validator.get_count_of_files_failed_to_validate() == 0
    ):
        exit(0)  # at least one file has been validated and no failures
    else:
        exit(1)  # exit code 1 to indicate generic error from script