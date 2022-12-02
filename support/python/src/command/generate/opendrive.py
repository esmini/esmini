from jinja2 import Environment,FileSystemLoader,select_autoescape
import xml.etree.ElementTree as ET
import json
import os
from support.python.src.globals import ESMINI_DIRECTORY_SUPPORT


class OpenDrive:
    def generate_j2(data,output):
        outputfolder = os.path.join(ESMINI_DIRECTORY_SUPPORT,"generated")
        if not os.path.exists(outputfolder):
            os.mkdir(outputfolder)
        output_file = os.path.join(outputfolder,output+".hpp")

        #Dump dicts for testing/fault tracing
        with open(output_file+".json",mode="w",encoding="utf-8") as file:
                json.dump(data,file,indent=4)
                file.close()

        template_file = os.path.join(ESMINI_DIRECTORY_SUPPORT,"jinja","hpp_template.j2")
        env = Environment(autoescape=select_autoescape(),loader=FileSystemLoader(""),trim_blocks=True,lstrip_blocks=True)
        template = env.get_template(template_file)
        content = template.render(data)
        with open(output_file,mode="w",encoding="utf-8") as message:
            message.write(content)
            print(f"generated file: {output}")


    def parser(file,name):
        parsed_data ={}
        tree = ET.parse(file)
        root = tree.getroot()
        OpenDrive.parse_children(root,parsed_data)
        return {"name":name,"data":parsed_data}


    def parse_children(parent,data=dict):
        for child in parent:
            if "complexType" in child.tag or "simpleType" in child.tag:
                child_sub_dict = OpenDrive.parse_children(child,{})
                name = child.attrib["name"]
                if name.startswith("t_"):
                    name = "class_"+name[2:]
                elif name.startswith("e_"):
                    name = "enum_"+name[2:]
                data.update({name:child_sub_dict})
            elif "extension" in child.tag or "restriction" in child.tag:
                base = child.attrib["base"]
                if base == "xs:string":
                    base = "string"
                data.update({"base":{base:OpenDrive.parse_children(child,{})}})
            elif "enumeration" in child.tag:
                value = child.attrib["value"]
                value = value.replace(" ","_")
                data.update({value:""})
            elif "element" in child.tag:
                data.update({child.attrib["name"]:child.attrib})
            elif "attribute" in child.tag:
                data.update({child.attrib["name"]:child.attrib})
            else:
                OpenDrive.parse_children(child,data)
        return data


    def generate_opendrive():
        path=""
        files_to_generate =[
            (os.path.join(path,"opendrive_17_core.xsd"),"Core"),
            (os.path.join(path,"opendrive_17_road.xsd"),"Road"),
            (os.path.join(path,"opendrive_17_lane.xsd"),"Lane"),
            (os.path.join(path,"opendrive_17_junction.xsd"),"Junction")
        ]
        for opendrive,output in files_to_generate:
            print("generating: ",output)
            with open(opendrive,mode="r",encoding="utf-8") as input:
                data = OpenDrive.parser(input,output)
            OpenDrive.generate_j2(data,output)
