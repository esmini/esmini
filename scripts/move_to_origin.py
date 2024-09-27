## Translate a scenario and referenced road network to origin
##
## When a scenario plays out far away from origin, like more than 100 km, 32 bit floating point numbers
## which is used in esmini lib and dat files, will loose precision. The result is shaky trajectories.
##
## This script solves that problem by moving the complete scenario closer to origin, by an offset
## selected from the starting point of the first geometry in the referenced OpenDRIVE file.
##
## The output is modified OpenSCENARIO and OpenDRIVE files with the offset appended to the filename.
## Original files are preserved.
##
## Note: Any SceneGraphFile (3D model) specified in the OpenSCENARIO file will not be handled. See
## https://esmini.github.io/#_openscenegraph_and_3d_models for example how to translate 3D model files.
##
## Usage: move_to_origin.py <OpenSCENARIO file>

import os
import xml.etree.ElementTree as etree
import sys
import argparse

parser = argparse.ArgumentParser(prog=os.path.basename(sys.argv[0]), description='Move scenario and road network to origin')
parser.add_argument('input_filename', help='required input OpenSCENARIO file')
parser.add_argument('output_filename', nargs='?', help='optional output filename')
args = parser.parse_args()

def append_string_to_filename_stem(str, filename):
    stem, ext = os.path.splitext(filename)
    return stem + str + ext

# Open OpenSCENARIO file to find out referenced OpenDRIVE file
osc = args.input_filename
osc_tree = etree.parse(osc)
odr_file_element = osc_tree.find('.//RoadNetwork/LogicFile')
odr = odr_file_element.attrib['filepath']

# Open referenced OpenDRIVE file and extract offset as x, y position of first geometry
# try both filepath as is and at the location of the OpenSCENARIO file
odr_tree = None
for file_path in [odr, os.path.split(osc)[0] + '/' + odr]:
    try:
        odr_tree = etree.parse(file_path)
        odr = file_path
        print('Found', file_path)
        break
    except:
        pass

if odr_tree is None:
    print('Failed to open', odr)
    exit(-1)

geom = odr_tree.find(".//road/planView/geometry")
offset = [-int(float(geom.attrib['x'])), -int(float(geom.attrib['y']))]
offset_str = '_offset_' + str(offset[0]) + '_' + str(offset[1])

# update position of all OpenDRIVE road geometries and signal with inertial position
for element in odr_tree.findall(".//road/planView/geometry") + odr_tree.findall(".//signal/positionInertial"):
    element.attrib['x'] = str(float(element.attrib['x']) + offset[0])
    element.attrib['y'] = str(float(element.attrib['y']) + offset[1])

# format OpenDRIVE xml and write the manipulated file, adding "_offset_x_y" to the original filename
etree.indent(odr_tree, space='   ', level=0)
odr_out_filename = append_string_to_filename_stem(offset_str, odr)
odr_tree.write(odr_out_filename)
print('Updated OpenDRIVE written to {0}'.format(odr_out_filename))

# Back to the OpenSCENARIO file, first update OpenDRIVE reference
odr_file_element.attrib['filepath'] = append_string_to_filename_stem(offset_str, odr)

# update position of all elements containing x and y coordinates
elements = osc_tree.findall('.//Center') + osc_tree.findall('.//WorldPosition')
for element in elements:
    element.attrib['x'] = str(float(element.attrib['x']) + offset[0])
    element.attrib['y'] = str(float(element.attrib['y']) + offset[1])

if args.output_filename is not None:
    osc_out_filename = args.output_filename
else:
    # add "_offset_x_y" to the original filename
    osc_out_filename = append_string_to_filename_stem(offset_str, osc)

# write the manipulated OpenSCENARIO file
osc_tree.write(osc_out_filename)
print('Updated OpenSCENARIO written to {0}'.format(osc_out_filename))
