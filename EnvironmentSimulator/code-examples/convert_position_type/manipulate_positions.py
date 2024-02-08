## Convert any world positions in trajectories to lane position type
##
## This script demonstrates how esmini RoadManager can be used to manipulate positions
## in an OpenSCENARIO file. E.g. converting to lane or road positions to make the scenario
## less dependent on specific road network. In other words making the scenario portable.
##
## Usage example, run from esmini/bin folder as:
##   ../EnvironmentSimulator/code-examples/convert_position_type/manipulate_positions.py ../resources/xosc/lane-change_trajectory_wp.xosc ../resources/xodr/straight_500m.xodr
## or if .py is not associated with Python:
##   python ../EnvironmentSimulator/code-examples/convert_position_type/manipulate_positions.py ../resources/xosc/lane-change_trajectory_wp.xosc ../resources/xodr/straight_500m.xodr
## or perhaps (typically on linux):
##   python3 ../EnvironmentSimulator/code-examples/convert_position_type/manipulate_positions.py ../resources/xosc/lane-change_trajectory_wp.xosc ../resources/xodr/straight_500m.xodr

import os
import argparse
import xml.etree.ElementTree as etree
import ctypes as ct
import sys

if sys.platform == "linux" or sys.platform == "linux2":
    rm = ct.CDLL("../bin/libesminiRMLib.so")
elif sys.platform == "darwin":
    rm = ct.CDLL("../bin/libesminiRMLib.dylib")
elif sys.platform == "win32":
    rm = ct.CDLL("../bin/esminiRMLib.dll")
else:
    print("Unsupported platform: {}".format(sys.platform))
    sys.exit(-1)

# Definition of RM_PositionData struct - should match esminiRMLib::RM_PositionData struct
class RM_PositionData(ct.Structure):
    _fields_ = [
        ("x", ct.c_float),
        ("y", ct.c_float),
        ("z", ct.c_float),
        ("h", ct.c_float),
        ("p", ct.c_float),
        ("r", ct.c_float),
        ("h_relative", ct.c_float),
        ("road_id", ct.c_int),
        ("junction_id", ct.c_int),  # -1 if not in a junction
        ("lane_id", ct.c_int),
        ("lane_offset", ct.c_float),
        ("s", ct.c_float),
    ]

# Specify argument types to a few esmini RoadManager functions
rm.RM_SetWorldPosition.argtypes = [ct.c_int, ct.c_float, ct.c_float, ct.c_float, ct.c_float, ct.c_float, ct.c_float]
rm.RM_SetWorldXYHPosition.argtypes = [ct.c_int, ct.c_float, ct.c_float, ct.c_float]

# specify necessary arguments
parser = argparse.ArgumentParser(description='Convert any trajectory world position into lane position')
parser.add_argument('xosc', help='OpenSCENARIO filename')
parser.add_argument('xodr', help='OpenDRIVE filename')
args = parser.parse_args()


# Open OpenSCENARIO file and modify position types
maneuver_tree = etree.parse(args.xosc)

# Initialize esmini RoadManger with given OpenDRIVE file
if rm.RM_Init(args.xodr.encode()) == -1:   # encode() converts string to pure byte array
    print("Failed to load OpenDRIVE file ", )
    sys.exit(-1)

# define a few objects that we'll need
rm_pos = rm.RM_CreatePosition()  # create a position object, returns a handle
rm_pos_data = RM_PositionData()  # object that will be passed and filled in with position info

# Collect all trajectories
trajectories = maneuver_tree.findall(".//Trajectory")

# Parse and manipulate the positions within the trajectory vertices
for trajectory in trajectories:
    print('Processing trajectory \"{}\":'.format(trajectory.attrib['name']))
    positions = trajectory.findall(".//Position")
    for pos in positions:
        # grab world position from the trajectory vertex
        w_pos = pos.find(".//WorldPosition")

        if w_pos is None:
            print('Warning: Found a non world position, skipping trajectory')
            continue

        if ('x' in w_pos):
            print('yes')
        if w_pos.get('x') is None or w_pos.get('y') is None or w_pos.get('h') is None:
            print('Error: Need all attributes \'x\', \'y\' and \'h\' in trajectory world position')
            exit(-1)

        x = float(w_pos.attrib['x'])
        y = float(w_pos.attrib['y'])
        h = float(w_pos.attrib['h'])

        # set world position in road context
        rm.RM_SetWorldXYHPosition(rm_pos, x, y, h)

        # extract lane position
        rm.RM_GetPositionData(rm_pos, ct.byref(rm_pos_data))
        print('   roadId {} laneId {} offset {:.2f} s {:.2f}'.format(
            rm_pos_data.road_id, rm_pos_data.lane_id, rm_pos_data.lane_offset, rm_pos_data.s))

        # update XML with the new position
        pos.remove(w_pos)
        l_pos = etree.SubElement(pos, 'LanePosition',
                                roadId = str(rm_pos_data.road_id),
                                laneId = str(rm_pos_data.lane_id),
                                s = str(rm_pos_data.s),
                                offset = str(rm_pos_data.lane_offset))

# write the manipulated file, adding "_lanepos" to the original filename, preserving the original file
basename, extension = os.path.splitext(args.xosc)
etree.indent(maneuver_tree, space='   ', level=0)
out_filename = basename + '_lanepos' + extension
maneuver_tree.write(out_filename)
print('Updated scenario written to', out_filename)
