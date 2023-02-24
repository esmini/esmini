'''
   demonstrate use of VariableConditions for triggering of lane changes from an external application

   Instruction:
     - make sure the esmini shared library (esminiLib.dll or esminiLib.so) is present in esmini/bin folder
     - if not, either compile esmini (see User guide) or fetch bin package release
     - from this folder (where this code module is), run: ./trig_lane_change_simple.py
'''

import sys
import ctypes


if sys.platform == "linux" or sys.platform == "linux2":
    se = ctypes.CDLL("../../../bin/libesminiLib.so")
elif sys.platform == "darwin":
    se = ctypes.CDLL("../../../bin/libesminiLib.dylib")
elif sys.platform == "win32":
    se = ctypes.CDLL("../../../bin/esminiLib.dll")
else:
    print("Unsupported platform: {}".format(sys.platform))
    quit()

se.SE_GetSimulationTime.restype = ctypes.c_float

se.SE_AddPath(b"../../../resources/xosc/Catalogs/Vehicles")

if se.SE_Init(b"./trig_lane_change.xosc", 0, 1, 0, 0) != 0:
    exit(-1)

state = 0
while se.SE_GetQuitFlag() != 1:

    if state == 0 and se.SE_GetSimulationTime() > 2:
        se.SE_SetVariableBool(b"ChangeLeft", True)
        state = 1

    elif state == 1 and se.SE_GetSimulationTime() > 5:
        se.SE_SetVariableBool(b"ChangeRight", True)
        state = 2

    se.SE_Step()

se.SE_Close()