from jinja2 import Environment, FileSystemLoader, select_autoescape
import xml.etree.ElementTree as ET
import json
import os
from support.python.src.globals import ESMINI_DIRECTORY_SUPPORT, ESMINI_DIRECTORY_ROOT
from support.python.src.formatter import format_green, format_yellow


class OpenDrive:
    def generate_file(self, data, output):
        """
        Handles the generation of the .json and .hpp files.

        Inputs:
        ---------
        data (dict): dictionary used in jinja generation
        output (str): name of file to generate
        """
        outputfolder = os.path.join(ESMINI_DIRECTORY_SUPPORT, "generated")
        if not os.path.exists(outputfolder):
            os.mkdir(outputfolder)
        output_file = os.path.join(outputfolder, output + ".hpp")

        # Dump dict to json for testing
        # self.print_dict(output_file, data)

        # Handles the generation of shared.hpp to fix mutual inclusion problem
        for key, value in data["data"].items():
            if "struct e_countryCode" in key:
                shared = {}
                shared["name"] = "shared"
                shared["namespace"] = data["name"]  # Get name/namespace
                shared["version"] = data["version"]  # Get version
                shared["data"] = {key: value}  # Copy e_countryCode
                self.create_exception_shared_hpp(outputfolder, shared)
                data["data"].pop(key)  # Remove e_countryCode to not generate it twice
                break

        # Generate the hpp file
        self.create_hpp_files(output_file, data)

    def create_exception_shared_hpp(self, outputfolder, data):
        """
        Generate a exception shared header for "struct e_countryCode" \n
        to fix a problem with mutual inclusion between signal and road

        Inputs:
        ---------
        outputfolder (str): name of folder to generate
        data (dict): dictionary used in jinja generation

        """
        template_folder = os.path.join(ESMINI_DIRECTORY_SUPPORT, "jinja")
        template_file = "shared_template.j2"
        env = Environment(
            autoescape=select_autoescape(),
            loader=FileSystemLoader(template_folder),
            trim_blocks=True,
            lstrip_blocks=True,
        )
        template = env.get_template(template_file)
        content = template.render(data)
        output_file = os.path.join(outputfolder, "Shared.hpp")
        with open(output_file, mode="w", encoding="utf-8") as message:
            message.write(content)
        filename = output_file.split("/")[-1]
        print(format_yellow(f"Generated exception file: {filename}"))

    def create_hpp_files(self, output_file, data):
        """
        Generates the .hpp files using jinja2.

        Inputs:
        ---------
        output_file (path): the name/location of file to generate
        data (dict): dictionary used in jinja generation
        """
        template_folder = os.path.join(ESMINI_DIRECTORY_SUPPORT, "jinja")
        template_file = "hpp_template.j2"
        env = Environment(
            autoescape=select_autoescape(),
            loader=FileSystemLoader(template_folder),
            trim_blocks=True,
            lstrip_blocks=True,
        )
        template = env.get_template(template_file)
        content = template.render(data)
        with open(output_file, mode="w", encoding="utf-8") as message:
            message.write(content)
        filename = output_file.split("/")[-1]
        print(format_green(f"Generated: {filename}"))

    def print_dict(self, output_file, data):
        """
        Generates the .json, primarly for debugging/testing.\n
        Dumps the data dictionary to a json file

        Inputs:
        ---------
        output_file (path): the name/location of file to generate
        data (dict): dictionary used in jinja generation
        """
        with open(output_file + ".json", mode="w", encoding="utf-8") as file:
            json.dump(data, file, indent=4)
            file.close()
        print((f"Printed dictionary"))

    def parser(self, file, name, version):
        """
        Parses a .xsd file in to a dictionary for jinja generation.
        Also do all necessary post-parsing processing of the dictionary.

        Inputs:
        ---------
        file (path): the path to file to be parsed
        name (str): name of file to be generated

        Returns:
        ---------
        (ref_list,dict) <- containing the parsed and processed file and all refences from the file
        """
        parsed_data = {}
        tree = ET.parse(file)
        root = tree.getroot()

        self.parse_children(root, parsed_data)
        parsed_data = self.union_to_struct(parsed_data)
        ordered = False
        while not ordered:
            old_dict = parsed_data.copy()
            parsed_data = self.order_dictionary(parsed_data)
            check = True
            for oldkey, newkey in zip(old_dict.keys(), parsed_data.keys()):
                if oldkey != newkey:
                    check = False
            ordered = check
        ref_list = self.create_ref_list([], name, parsed_data)
        parsed_dict = {"name": name, "version": version, "data": parsed_data}
        return (ref_list, parsed_dict)

    def create_ref_list(self, ref_list, path, data):
        """
        Create a list containing all references from all files.
        reference example -> namespace::struct::enum

        Inputs:
        ---------
        ref_list (list): list to append all references
        path (str): path/name of dict to be checked
        data (dict): dict to extract references from

        Returns:
        ---------
        ref_list <- containing appended references
        """
        for key, value in data.items():
            if not isinstance(value, str):
                if "class" in key or "struct" in key:
                    key = key.split(" ")[-1]
                    temp_path = path + "::" + key
                    ref_list.append(temp_path)
                    self.create_ref_list(ref_list, temp_path, value)
        return ref_list

    def order_dictionary(self, dict_to_order):
        """
        Orders dictionary after order of class declaration needed for c++ code. \n
        Order: first enums and structs, then classes ordered according to get_key_order().

        Inputs:
        ---------
        dict_to_order (dict): dictionary to be ordered

        Returns:
        ---------
        ordered_dict <- containing the ordered dict
        """
        ordered_dict = {}
        keys = dict_to_order.keys()
        first_keys = []
        for key in keys:
            if "enum" in key or "struct" in key:  # Extract enums and structs
                first_keys.append(key)
        for key in first_keys:  # Add Enums and structs (items) first
            ordered_dict.update({key: dict_to_order.pop(key)})
        # Update keys after enums/structs has been extracted
        keys = dict_to_order.keys()
        # Fetch the correct order
        ordered_key_list = self.get_key_order(dict_to_order, [], keys)

        for key in ordered_key_list:  # Add classes (items) after correct order
            ordered_dict.update({"class " + key: dict_to_order.pop("class " + key)})

        for key, value in dict_to_order.items():  # Add all items left in unordered dict
            ordered_dict.update({key: value})
        return ordered_dict

    def get_key_order(self, dict_to_order, order, keys):
        """
        Creates a list containing the order for keys. \n
        Will return order after what types is under public/private variables accoring to c++.\n
        Ex: If class A has class B as a variable, B should be before of A.

        Inputs:
        ---------
        dict_to_order (dict): dictionary to be ordered
        order (list): list to append keys to
        keys (list): list containing all keys of dict_to_order

        Returns:
        ---------
        list <- containing the order of keys
        """

        for key, value in dict_to_order.items():
            if isinstance(value, dict) and "type" in value.keys():
                if isinstance(value["type"], str) and value["type"] != key:
                    if value["type"] in keys:
                        if value["type"] not in order:
                            order.append(value["type"])
                    if "std::vector" in value["type"]:
                        temp_str = value["type"]
                        temp_str = temp_str.replace("std::vector<", "")
                        temp_str = temp_str.replace(">", "")
                        if temp_str in keys:
                            if temp_str not in order:
                                order.append(temp_str)
            if not isinstance(value, str):
                self.get_key_order(value, order, keys)
        return order

    def find_core_reference(self, ref_list, dict_to_check, current_file):
        """
        Checks dictionary for types with ref to other files, fixes the namespace.

        Inputs:
        ---------
        dict_to_check (dict): dictionary to be checked
        current_file (str): name of the current file

        Returns:
        ---------
        updated_dict <- containing the updated and checked dict
        """
        updated_dict = {}
        for key, value in dict_to_check.items():
            for refrence in ref_list:
                if refrence.split("::")[0] != current_file:
                    if isinstance(value, dict) and "type" in value.keys():
                        if isinstance(value["type"], str):
                            if value["type"] == refrence.split("::")[-1]:
                                value["type"] = refrence
                                break
                            if "std::vector" in value["type"]:
                                temp_str = value["type"]
                                temp_str = temp_str.replace("std::vector<", "")
                                temp_str = temp_str.replace(">", "")
                                if temp_str == refrence.split("::")[-1]:
                                    value["type"] = "std::vector<" + refrence + ">"
                                    break
            if not isinstance(value, str):
                value.update(self.find_core_reference(ref_list, value, current_file))
            updated_dict.update({key: value})
        return updated_dict

    def union_to_struct(self, data):
        """
        Restructures a dictionary containing the xsd keyword union to
        fit the structure of struct.

        Inputs:
        ---------
        data (dict): dictionary to be fixed

        Returns:
        ---------
        dict <- containing the restructerd dict
        """
        new_dict = {}
        structs = []
        # get all structs
        for item in data.items():
            if "struct" in item[0]:
                structs.append(item)
        # create correct dictionaries for each struct
        struct_members = []
        for key, value in structs:
            members_dict = {}
            for _, value1 in value.items():
                for _, value2 in value1.items():
                    members = value2.split(" ")
                    for member in members:
                        for data_key in data.keys():
                            if member in data_key:
                                struct_members.append(data_key)
                                members_dict.update({data_key: data.get(data_key)})
                        # special case if struct contains values for core file
                        if (
                            member == "t_grEqZero" or member == "t_grZero"
                        ):  # TODO these comes from core.xsd
                            # Remove struct t_ to get the name
                            member_name = "double " + key[9:]
                            members_dict.update({member_name: ""})
            new_dict.update({key: members_dict})
        # copy all other datastructures to the new dict
        # expect the ones that exists in the newly created structs
        for key, value in data.items():
            if key in struct_members or "struct" in key:
                pass
            else:
                new_dict.update({key: value})
        return new_dict

    def parse_children(self, parent, data):
        """
        Parses the xml.tree and extracts necessary data for jinja2 generation.

        Inputs:
        ---------
        parent (xml.tree): xml to be parsed
        data (dict): dictionary to append parsed data to

        Returns:
        ---------
        data (dict)
        """
        attributes_dict = {}
        for child in parent:
            if "complexType" in child.tag:  # -> class
                child_sub_dict = self.parse_children(child, {})
                name = child.attrib["name"]
                name = "class " + name
                data.update({name: child_sub_dict})

            elif "simpleType" in child.tag:  # -> struct/enum/variable
                child_sub_dict = self.parse_children(child, {})
                name = child.attrib["name"]
                if "union" in child_sub_dict.keys():
                    name = "struct " + name
                if "base" in child_sub_dict.keys():
                    sub_sub_dict = child_sub_dict["base"]
                    for key, value in sub_sub_dict.items():
                        if value == {}:
                            name = key + " " + name
                        if key == "enum":
                            name = "enum class " + name
                data.update({name: child_sub_dict})

            elif "extension" in child.tag or "restriction" in child.tag:
                base = child.attrib["base"]
                sub_children = self.parse_children(child, {})
                if "enum" in sub_children.values():
                    base = "enum"
                base = self.xsd_to_cpp_types(base)
                data.update({"base": {base: sub_children}})

            elif "sequence" in child.tag:
                data.update({"sequence": self.parse_children(child, {})})

            elif "enumeration" in child.tag:  # enum elements
                value = child.attrib["value"]
                value = self.fix_illegal_chars(value)
                data.update({value: "enum"})

            elif "element" in child.tag:  # Public variables
                attributes = child.attrib
                child.attrib["name"] = self.fix_illegal_names(child.attrib["name"])
                if "type" in attributes:
                    attributes["type"] = self.xsd_to_cpp_types(attributes["type"])
                if "maxOccurs" in attributes:
                    if attributes["maxOccurs"] == "unbounded":
                        attributes["type"] = "std::vector<" + attributes["type"] + ">"
                data.update({child.attrib["name"]: attributes})

            elif "attribute" in child.tag:  # Private variables
                sub_dict = self.parse_children(child, {})
                doc = ""
                if "docs" in sub_dict:
                    doc = sub_dict["docs"]
                attributes = child.attrib
                if len(attributes) > 1:
                    attributes["type"] = self.xsd_to_cpp_types(attributes["type"])
                attributes.update({"docs": doc})
                attributes_dict.update({child.attrib["name"]: attributes})

            elif "union" in child.tag:  # structs
                data.update({"union": child.attrib})

            elif "documentation" in child.tag:  # extracts documentation from xsd
                data.update({"docs": child.text})

            else:
                self.parse_children(child, data)

        if len(attributes_dict) != 0:
            data.update({"attributes": attributes_dict})
        return data

    def xsd_to_cpp_types(self, type):
        """
        Replaces xsd types with corresponding c++ types.

        Inputs:
        ---------
        type (str): str containing the type

        Returns:
        ---------
        attribute (str) <- with the corresponding c++ type
        """
        if type == "xs:string":
            type = "std::string"
        elif type == "xs:double":
            type = "double"
        elif type == "xs:boolean":
            type = "bool"
        elif type == "xs:decimal":
            type = "double"
        elif type == "xs:integer":
            type = "int"
        elif type == "xs:negativeInteger":
            type = "int"
        elif type == "xs:nonNegativeInteger":  # TODO size_t or int?
            type = "size_t"
        elif type == "xs:positiveInteger":  # TODO size_t or int?
            type = "size_t"
        elif type == "xs:float":
            type = "float"
        elif type == "t_grEqZero":  # TODO should be fetch or linked to core file
            type = "double"
        elif type == "t_grZero":  # TODO should be fetch or linked to core file
            type = "double"
        elif type == "t_zeroOne":  # TODO should be fetch or linked to core file
            type = "double"
        elif type == "t_bool":
            type = "bool"
        return type

    def fix_illegal_names(self, string):
        """
        Replace illegal (c++) names.

        Inputs:
        ---------
        string (str): string to be checked

        Returns:
        ---------
        string (str) <- without illegal name
        """
        if string == "explicit":  # TODO agree on naming
            string = "exp"
        if string == "switch":  # TODO agree on naming
            string = "sw"
        return string

    def fix_illegal_chars(self, string):
        """
        Replace illegal(c++) chars from string.

        Inputs:
        ---------
        string (str): string to be checked

        Returns:
        ---------
        string (str) <- without illegal chars
        """
        string = string.replace("/", "")
        string = string.replace(" ", "_")
        string = string.replace("+", "positive")  # TODO agree on naming
        string = string.replace("-", "negative")  # TODO agree on naming
        string = string.replace("%", "percent")  # TODO agree on naming
        return string

    def generate_opendrive(self, version):
        """
        Generates all opendrive files.

        Inputs:
        ---------
        version (str): opendrive version
        """
        f_version = version.replace(".", "")  # fix naming for files (X.X) -> (XX)
        opendrive_schema_path = os.path.join(
            ESMINI_DIRECTORY_ROOT, "..", "OpenDrive_" + f_version
        )
        files_to_generate = [
            (
                os.path.join(
                    opendrive_schema_path, "opendrive_" + f_version + "_core.xsd"
                ),
                "Core",
            ),
            (
                os.path.join(
                    opendrive_schema_path, "opendrive_" + f_version + "_road.xsd"
                ),
                "Road",
            ),
            (
                os.path.join(
                    opendrive_schema_path, "opendrive_" + f_version + "_lane.xsd"
                ),
                "Lane",
            ),
            (
                os.path.join(
                    opendrive_schema_path, "opendrive_" + f_version + "_junction.xsd"
                ),
                "Junction",
            ),
            (
                os.path.join(
                    opendrive_schema_path, "opendrive_" + f_version + "_object.xsd"
                ),
                "Object",
            ),
            (
                os.path.join(
                    opendrive_schema_path, "opendrive_" + f_version + "_signal.xsd"
                ),
                "Signal",
            ),
            (
                os.path.join(
                    opendrive_schema_path, "opendrive_" + f_version + "_railroad.xsd"
                ),
                "Railroad",
            ),
        ]
        list_of_parsed_dict = []
        reference_list = []
        for opendrive, name in files_to_generate:
            with open(opendrive, mode="r", encoding="utf-8") as input:
                ref_list, parsed_dict = self.parser(input, name, version)
                list_of_parsed_dict.append(parsed_dict)
                reference_list = reference_list + ref_list

        for dict in list_of_parsed_dict:
            dict["data"] = self.find_core_reference(
                reference_list, dict["data"], dict["name"]
            )
            self.generate_file(dict, dict["name"])
