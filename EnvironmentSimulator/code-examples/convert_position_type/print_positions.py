## Print entity positions as lane position
##
## This script demonstrates how esmini can be used to extract positions in any type
##
## Usage example, run from esmini/bin folder as:
##   ../EnvironmentSimulator/code-examples/convert_position_type/print_positions.py ../resources/xosc/trajectory-test.xosc
## or:
##   python ../EnvironmentSimulator/code-examples/convert_position_type/print_positions.py ../resources/xosc/trajectory-test.xosc
## or perhaps (typically on linux):
##   python3 ../EnvironmentSimulator/code-examples/convert_position_type/print_positions.py ../resources/xosc/trajectory-test.xosc


import argparse
import ctypes
from sys import platform

if platform == "linux" or platform == "linux2":
    se = ctypes.CDLL("../bin/libesminiLib.so")
elif platform == "darwin":
    se = ctypes.CDLL("../bin/libesminiLib.dylib")
elif platform == "win32":
    se = ctypes.CDLL("../bin/esminiLib.dll")
else:
    print("Unsupported platform: {}".format(platform))
    quit()

# Definition of SE_ScenarioObjectState struct
class SEScenarioObjectState(ctypes.Structure):
    _fields_ = [
        ("id", ctypes.c_int),
        ("model_id", ctypes.c_int),
        ("control", ctypes.c_int),
        ("timestamp", ctypes.c_float),
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float),
        ("h", ctypes.c_float),
        ("p", ctypes.c_float),
        ("r", ctypes.c_float),
        ("roadId", ctypes.c_int),
        ("junctionId", ctypes.c_int),
        ("t", ctypes.c_float),
        ("laneId", ctypes.c_int),
        ("laneOffset", ctypes.c_float),
        ("s", ctypes.c_float),
        ("speed", ctypes.c_float),
        ("centerOffsetX", ctypes.c_float),
        ("centerOffsetY", ctypes.c_float),
        ("centerOffsetZ", ctypes.c_float),
        ("width", ctypes.c_float),
        ("length", ctypes.c_float),
        ("height", ctypes.c_float),
        ("objectType", ctypes.c_int),
        ("objectCategory", ctypes.c_int),
        ("wheelAngle", ctypes.c_float),
        ("wheelRot", ctypes.c_float),
    ]

# specify argument type for the SE_StepDT function
se.SE_StepDT.argtypes = [ctypes.c_float]

# specify necessary argument
parser = argparse.ArgumentParser(description='Print entity positions as lane position')
parser.add_argument('xosc', help='OpenSCENARIO filename')
args = parser.parse_args()

# Initialize and run the scenario
if se.SE_Init(args.xosc.encode(), 0, 1, 0, 0) != 0:
    print('Error: Failed to initialize the scenario')
    exit(-1)

obj_state = SEScenarioObjectState()  # object that will be passed and filled in with object state info

while se.SE_GetQuitFlag() == 0:
    for j in range(se.SE_GetNumberOfObjects()):
        se.SE_GetObjectState(se.SE_GetId(j), ctypes.byref(obj_state))
        print('Time {:.2f} ObjId {} roadId {} laneId {} s {:.2f} laneOffset {:.2f} s {:.2f} x {:.2f} y {:.2f} heading {:.2f}'.format(
            obj_state.timestamp, obj_state.id, obj_state.roadId, obj_state.laneId, obj_state.s, obj_state.laneOffset,
            obj_state.s, obj_state.x, obj_state.y, obj_state.h))
        # print('WorldPosition x="{:.2f}" y="{:.2f}" h="{:.2f}"'.format(obj_state.x, obj_state.y, obj_state.h))
    se.SE_StepDT(0.1)
