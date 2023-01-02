from jinja2 import Environment, FileSystemLoader, select_autoescape
import xml.etree.ElementTree as ET
import json
import os

from support.python.src.command.run.run import Run
from support.python.src.globals import (
    ESMINI_DIRECTORY_EXTERNALS,
    ESMINI_DIRECTORY_JINJA_TEMPLATES,
    ESMINI_DIRECTORY_SUPPORT,
)
from support.python.src.formatter import format_green, format_yellow, format_red


class OpenDrive:
    def generate_file(self, data, output_folder):
        """
        Handles the generation of the .json and .hpp files.

        Inputs:
        ---------
        data (dict): dictionary used in jinja generation
        output (str): name of file to generate
        """

        if not os.path.exists(output_folder):
            os.mkdir(output_folder)
        output_file = os.path.join(output_folder, data["name"] + ".hpp")

        # Dump dict to json for testing
        # self.print_dict(output_file, data)

        # Handles the generation of shared.hpp to fix mutual inclusion problem
        if data["name"] == "Road":
            data = self.create_exception_shared(
                data, os.path.join(output_folder, "Shared.hpp")
            )

        # Generate the hpp file
        template_file = "hpp_template.j2"
        self.create_hpp_file(template_file, output_file, data)

    def create_exception_shared(self, data, output_file):
        """
        Creates a exception shared dictionary for "struct e_countryCode"
        and generates a shared hpp file for the dictionary.\n
        This to fix a problem with mutual inclusion between signal and road

        Inputs:
        ---------
        dictionary (dict): the dictionary containing all parsed data
        outputfolder (path): to the folder where the file should be generated
        Returns:
        ---------
        dict <- with the "shared" object removed
        """
        for key, value in data["data"].items():
            if "struct e_countryCode" in key:
                shared = {}
                shared["name"] = "shared"
                shared["namespace"] = data["name"]  # Get name/namespace
                shared["version"] = data["version"]  # Get version
                shared["data"] = {key: value}  # Copy e_countryCode
                self.create_hpp_file("shared_template.j2", output_file, shared)
                data["data"].pop(key)  # Remove e_countryCode to not generate it twice
                break
        return data

    def create_hpp_file(self, template_file, output_file, data):
        """
        Generates the .hpp files using jinja2.

        Inputs:
        ---------
        template_file (str): name of template to use for generation
        output_file (path): the name/location of file to generate
        data (dict): dictionary used in jinja generation
        """
        env = Environment(
            autoescape=select_autoescape(),
            loader=FileSystemLoader(ESMINI_DIRECTORY_JINJA_TEMPLATES),
            trim_blocks=True,
            lstrip_blocks=True,
        )
        template = env.get_template(template_file)
        content = template.render(data)
        try:
            with open(output_file, mode="w", encoding="utf-8") as message:
                message.write(content)
        except OSError as e:
            print(format_red(f"{e}"))

        print(format_green("Generated: " + output_file.split("/")[-1]))

    def print_dict(self, output_file, data):
        """
        Generates the .json, primarly for debugging/testing.\n
        Dumps the data dictionary to a json file

        Inputs:
        ---------
        output_file (path): the name/location of file to generate
        data (dict): dictionary used in jinja generation
        """
        try:
            with open(output_file + ".json", mode="w", encoding="utf-8") as file:
                json.dump(data, file, indent=4)
                file.close()
        except OSError as e:
            print(format_red(f"{e}"))
        print(format_yellow("Dumped dictionary"))

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
        tuple(ref_list,dict) <- (list) of all references from the file, (dict) of the parsed post-processed file
        """
        parsed_data = {}
        tree = ET.parse(file)
        root = tree.getroot()

        self.parse_children(root, parsed_data)
        parsed_data = self.union_to_struct(parsed_data)  # Create struct from unions
        ref_list = self.create_ref_list(
            [], name, parsed_data
        )  # Create list of references for parsed_data

        # Runs until dictionary is not changed between two runs -> ordered
        ordered = False
        while not ordered:
            old_dict = parsed_data.copy()
            parsed_data = self.order_dictionary(parsed_data)
            check = True
            for oldkey, newkey in zip(old_dict.keys(), parsed_data.keys()):
                if oldkey != newkey:
                    check = False
            ordered = check

        parsed_data = self.create_inheritance(
            parsed_data
        )  # Fix inheritance if classes has it
        parsed_dict = {"name": name, "version": version, "data": parsed_data}
        return (ref_list, parsed_dict)

    def get_inheritance(self, root):
        """
        Looks through the dictionary and searches for inheritances
            looks for the keyword ("choice") in dictionary to find inheritance

        Inputs:
        ---------
        root (dict):  dictionary to be searched

        Returns:
        ---------
        list(tuples) <- list of tuples containing (child,parent) for inheritance
        """
        inheritance = []
        for classname, classdict in root.items():
            if isinstance(classdict, dict):
                self.get_inheritance_rec(classdict, classname, inheritance)
        return inheritance

    def get_inheritance_rec(self, parent, classname, inheritance):
        """
        Looks through the dictionary and searches for inheritances
            looks for the keyword ("choice") in dictionary to find inheritance

        Inputs:
        ---------
        parent (dict):  dictionary to be searched
        classname (str):
        Returns:
        ---------
        list(tuples) <- list of tuples containing (childtype,classname) for inheritance
        """
        for child, contains in parent.items():
            if isinstance(contains, dict):
                if child == "choice":
                    for _, items in contains.items():
                        if "type" in items.keys():
                            inheritance.append((items["type"], classname))
                else:
                    self.get_inheritance_rec(contains, classname, inheritance)
        return inheritance

    def create_inheritance(self, root):
        """
        Creates the inheritance between elements in dictionary
            Add ("inherit") key to element in dictionary if element has inheritance

        Inputs:
        ---------
        root (dict):  dictionary to be searched

        Returns:
        ---------
        dict <- the updated dictionary with inheritance added
        """
        inheritance = self.get_inheritance(root)
        element_to_remove = []
        element_to_add = []
        for child, parent in inheritance:
            for key, value in root.items():
                if "class " + child == key:  # Check if key exists in inheritance
                    element_to_remove.append(key)  # Add to be removed
                    # append value with inherits key
                    value.update({"inherits": parent[6:]})
                    # Add new dict to list to append
                    element_to_add.append({key: value})
        if element_to_remove:  # Remove non-valid elements
            for element in element_to_remove:
                root.pop(element)
        if element_to_add:  # Add the newly created elements with inheritance
            for element in element_to_add:
                root.update(element)
        return root

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
                        # Removes "std::vector< >" to get type
                        temp_str = value["type"][12:-1]
                        if temp_str in keys and temp_str not in order:
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
                                # Removes "std::vector< > to get type"
                                temp_str = value["type"][12:-1]
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
                        # special case for doubles that inside structs
                        if member == "t_grEqZero" or member == "t_grZero":
                            # Replace "struct t_" with "double" in name
                            member_name = key.replace("struct t_", "double ")
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
            if "complexType" in child.tag:  # class
                child_sub_dict = self.parse_children(child, {})
                name = child.attrib["name"]
                name = "class " + name
                data.update({name: child_sub_dict})

            elif "simpleType" in child.tag:  # struct/enum/variable
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
                child_sub_dict = self.parse_children(child, {})
                base = child.attrib["base"]
                if "enum" in child_sub_dict.values():
                    base = "enum"
                base = self.xsd_to_cpp_types(base)
                data.update({"base": {base: child_sub_dict}})

            elif "sequence" in child.tag:
                child_sub_dict = self.parse_children(child, {})
                # if choice under sequence -> ignore choice
                if "choice" in child_sub_dict.keys():
                    choice_dict = child_sub_dict.pop("choice")
                    child_sub_dict.update(choice_dict)
                data.update({"sequence": child_sub_dict})

            elif "choice" in child.tag:  # Handles choice keyword
                child_sub_dict = self.parse_children(child, {})
                data.update({"choice": child_sub_dict})

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

            elif "attribute" in child.tag:  # Protected variables -> special case
                child_sub_dict = self.parse_children(child, {})
                doc = ""
                if "docs" in child_sub_dict:
                    doc = child_sub_dict["docs"]
                attributes = child.attrib
                if len(attributes) > 1:
                    attributes["type"] = self.xsd_to_cpp_types(attributes["type"])
                attributes.update({"docs": doc})
                attributes_dict.update({child.attrib["name"]: attributes})

            elif "union" in child.tag:  # structs
                data.update({"union": child.attrib})

            elif "documentation" in child.tag:  # extracts documentation
                data.update({"docs": child.text})

            else:
                self.parse_children(child, data)

        # Special case for attributes to create them under one key
        # When all attributes are looked through this will added all under one key
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
        elif (
            type == "xs:double"
            or type == "xs:decimal"
            or type == "t_grEqZero"
            or type == "t_zeroOne"
            or type == "t_grZero"
        ):  # TODO t_grEqZero,t_zeroOne,t_grZero should comes from core file
            type = "double"
        elif type == "xs:integer" or type == "xs:negativeInteger":
            type = "int"
        elif type == "xs:nonNegativeInteger" or type == "xs:positiveInteger":
            type = "size_t"
        elif type == "xs:float":
            type = "float"
        elif type == "t_bool" or type == "xs:boolean":
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
        if string == "explicit":
            string = "exp"  # TODO agree on naming
        if string == "switch":
            string = "sw"  # TODO agree on naming
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
        string = string.replace("+", "positive")
        string = string.replace("-", "negative")
        string = string.replace("%", "percent")
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
            ESMINI_DIRECTORY_EXTERNALS, "OpenDrive_" + f_version
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
            try:
                with open(opendrive, mode="r", encoding="utf-8") as input:
                    ref_list, parsed_dict = self.parser(input, name, version)
                    list_of_parsed_dict.append(parsed_dict)
                    reference_list = reference_list + ref_list
            except OSError as e:
                print(format_red(f"{e}"))

        output_folder = os.path.join(ESMINI_DIRECTORY_SUPPORT, "generated")
        for dict in list_of_parsed_dict:
            dict["data"] = self.find_core_reference(
                reference_list, dict["data"], dict["name"]
            )
            self.generate_file(dict, output_folder)

        # Run.run_clang_format([outputfolder],[],False)
