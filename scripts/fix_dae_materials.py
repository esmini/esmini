# This script adjust the material properties of .dae files exported from Blender
# for use in esmini

import os
import re
import argparse
import lxml.etree as ET
import time

# Fetch any arguments
parser = argparse.ArgumentParser(description='Populate ambient material properties based on diffuse values')
parser.add_argument('input_file', help='DAE file')

args = parser.parse_args()
outfile = args.input_file.replace('.dae', '_fixed.dae')

# Open and parse DAE file
parser = ET.XMLParser(remove_blank_text=True, huge_tree=True)
tree = ET.parse(args.input_file, parser)
materials = tree.findall('.//lambert', namespaces=tree.getroot().nsmap)

print('number of materials: {}'.format(len(materials)))


for mat in materials:

    diffuse = mat.find('.//diffuse', namespaces=tree.getroot().nsmap)

    if (diffuse is not None):
        color = diffuse.find('color', namespaces=tree.getroot().nsmap)
        if color is None:
            color = ET.Element('color')
            color.set('sid', 'ambient')
            color.text = "{} {} {} 1".format(0.5, 0.5, 0.5)

        ambient = mat.find('.//ambient', namespaces=tree.getroot().nsmap)
        if (ambient is not None):
            acolor = ambient.find('color', namespaces=tree.getroot().nsmap)
            acolor.text = color.text
        else:
            a = ET.Element('ambient')
            b = ET.Element('color')
            b.set('sid', 'ambient')
            b.text = color.text
            a.append(b)
            mat.append(a)

        reflectivity = mat.find('.//reflectivity', namespaces=tree.getroot().nsmap)
        if reflectivity is not None:
            shininess_value = 100 * float(reflectivity.find('float', namespaces=tree.getroot().nsmap).text)
            mat.remove(reflectivity)
        else:
            shininess_value = 5
        s = ET.Element('shininess')
        f = ET.Element('float')
        f.text = str(shininess_value)
        s.append(f)
        mat.append(s)

        specular = mat.find('.//specular', namespaces=tree.getroot().nsmap)
        if specular is not None:
            scolor = specular.find('color', namespaces=tree.getroot().nsmap)
            scolor.text = '0.5 0.5 0.5 1'
        else:
            a = ET.Element('specular')
            b = ET.Element('color')
            b.set('sid', 'specular')
            spec = min(1.0, 0.04 * shininess_value)
            print(shininess_value, 0.04*shininess_value)
            print("spec:", spec)
            b.text = "{} {} {} 1".format(spec, spec, spec)
            a.append(b)
            mat.append(a)

        ior = mat.find('.//index_of_refraction', namespaces=tree.getroot().nsmap)
        if ior is None:
            a = ET.Element('index_of_refraction')
            b = ET.Element('float')
            b.set('sid', 'index_of_refraction')
            b.text = '1.0'
            a.append(b)
            mat.append(a)

    mat.tag = 'phong'

tree.write(outfile, pretty_print=True)
