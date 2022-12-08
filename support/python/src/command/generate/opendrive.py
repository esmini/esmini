from jinja2 import Environment, FileSystemLoader, select_autoescape
import xml.etree.ElementTree as ET
import json
import os
from support.python.src.globals import ESMINI_DIRECTORY_SUPPORT, ESMINI_DIRECTORY_ROOT
from support.python.src.formatter import format_green,format_yellow


class OpenDrive:
    def generate_file(self, data, output):
        """
            Handles the generation of the .json and .hpp files

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
        #self.print_dict(output_file, data)

        # Generate the hpp file
        self.create_hpp_files(output_file, data)


    def create_hpp_files(self, output_file, data):
        """
            Generates the .hpp files using jinja2

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
            Generates the .json, primarly for debugging/testing\n
            Dumps the data dictionary to a json file

            Inputs:
            ---------
            output_file (path): the name/location of file to generate
            data (dict): dictionary used in jinja generation
        """
        with open(output_file + ".json", mode="w", encoding="utf-8") as file:
            json.dump(data, file, indent=4)
            file.close()
        print(format_yellow(f"Printed dictionary"))

    def parser(self, file, name,version):
        """
            Parses a .xsd file in to a dictionary

            Inputs:
            ---------
            file (path): the path to file to be parsed
            name (str): name of file to be generated

            Returns:
            ---------
            dict <- containing the parsed file
        """
        parsed_data = {}
        tree = ET.parse(file)
        root = tree.getroot()
        self.parse_children(root, parsed_data)
        parsed_data = self.union_to_struct(parsed_data)
        return {"name": name,"version":version, "data": parsed_data}

    def union_to_struct(self,data):
        """
            Restructures a dictionary containing the xsd keyword union to
            fit the structure of struct

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
        for key,value in structs:
            members_dict = {}
            for _,value1 in value.items():
                for _,value2 in value1.items():
                    members = value2.split(" ")
                    for member in members:
                        for data_key in data.keys():
                            if member in data_key:
                                struct_members.append(data_key)
                                members_dict.update({data_key:data.get(data_key)})
                        #special case if struct contains values for core file
                        if member == "t_grEqZero" or member == "t_grZero": #TODO these comes from core.xsd
                            #Remove struct t_ to get the name
                            member_name = "double "+key[9:]
                            members_dict.update({member_name:""})
            new_dict.update({key:members_dict})
        #copy all other datastructures to the new dict
        #expect the ones that exists in the newly created structs
        for key,value in data.items():
            if key in struct_members or "struct" in key:
                pass
            else:
                new_dict.update({key:value})
        return new_dict

    def parse_children(self, parent, data):
        """
            Parses the xml.tree and extracts necessary data for jinja2 generation

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
            if "complexType" in child.tag: #-> class
                child_sub_dict = self.parse_children(child, {})
                name = child.attrib["name"]
                name = "class " + name
                data.update({name: child_sub_dict})

            elif "simpleType" in child.tag: #-> struct/enum/variable
                child_sub_dict = self.parse_children(child, {})
                name = child.attrib["name"]
                if "union" in child_sub_dict.keys():
                    name = "struct " + name
                if "base" in child_sub_dict.keys():
                    sub_sub_dict = child_sub_dict["base"]
                    for key,value in sub_sub_dict.items():
                        if value == {}:
                            name = key + " " + name
                        if key == "enum":
                            name = "enum " + name
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

            elif "enumeration" in child.tag: #enum elements
                value = child.attrib["value"]
                value = self.fix_non_legal_chars(value)
                data.update({value:"enum"})

            elif "element" in child.tag: #Public variables
                attributes = child.attrib
                if "type" in attributes:
                    attributes["type"] = self.xsd_to_cpp_types(attributes["type"])
                if "maxOccurs" in attributes:
                    if attributes["maxOccurs"] == "unbounded":
                        attributes["type"] = "std::vector<"+attributes["type"]+">"
                data.update({child.attrib["name"]: attributes})

            elif "attribute" in child.tag: #Private variables
                sub_dict = self.parse_children(child,{})
                doc =""
                if "docs" in sub_dict:
                    doc = sub_dict["docs"]
                attributes = child.attrib
                if len(attributes) > 1:
                    attributes["type"] = self.xsd_to_cpp_types(attributes["type"])
                attributes.update({"docs":doc})
                attributes_dict.update({child.attrib["name"]:attributes})

            elif "union" in child.tag: # structs
                data.update({"union":child.attrib})

            elif "documentation" in child.tag: # extracts documentation from xsd
                data.update({"docs":child.text})

            else:
                self.parse_children(child, data)

        if len(attributes_dict) != 0:
            data.update({"attributes": attributes_dict})
        return data

    def xsd_to_cpp_types(self, type):
        """
            Replaces xsd types with corresponding cpp types

            Inputs:
            ---------
            type (str): str containing the type

            Returns:
            ---------
            attribute (str) <- with the corresponding cpp type
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
        elif type == "xs:float":
            type = "float"
        elif type == "t_grEqZero": # TODO should be fetch or linked to core file
            type = "double"
        elif type == "t_grZero": # TODO should be fetch or linked to core file
            type = "double"
        return type

    def fix_non_legal_chars(self, string):
        """
            Remove non legal chars from string

            Inputs:
            ---------
            string (str): string to be checked

            Returns:
            ---------
            string (str) <- without non legal chars
        """
        string = string.replace("/", "")
        string = string.replace(" ", "_")
        string = string.replace("+", "positive") #TODO agree on naming
        string = string.replace("-", "negative") #TODO agree on naming
        string = string.replace("%", "percent") #TODO agree on naming
        return string

    def generate_opendrive(self,version):
        """
            Generates all opendrive files
        """
        f_version = version.replace(".","") #fix naming for files (X.X) -> (XX)
        opendrive_schema_path = os.path.join(
            ESMINI_DIRECTORY_ROOT, "..", "OpenDrive_"+f_version
        )
        files_to_generate = [
            (os.path.join(opendrive_schema_path, "opendrive_"+f_version+"_core.xsd"), "Core"),
            (os.path.join(opendrive_schema_path, "opendrive_"+f_version+"_road.xsd"), "Road"),
            (os.path.join(opendrive_schema_path, "opendrive_"+f_version+"_lane.xsd"), "Lane"),
            (
                os.path.join(opendrive_schema_path, "opendrive_"+f_version+"_junction.xsd"),
                "Junction",
            ),
            (os.path.join(opendrive_schema_path, "opendrive_"+f_version+"_object.xsd"), "Object"),
            (os.path.join(opendrive_schema_path, "opendrive_"+f_version+"_signal.xsd"), "Signal"),
            (
                os.path.join(opendrive_schema_path, "opendrive_"+f_version+"_railroad.xsd"),
                "Railroad",
            ),
        ]
        for opendrive, output in files_to_generate:
            with open(opendrive, mode="r", encoding="utf-8") as input:
                data = self.parser(input, output,version)
            self.generate_file(data, output)

