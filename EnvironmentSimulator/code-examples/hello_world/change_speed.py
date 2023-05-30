'''
   demonstrate how to inject speed changes for a scenario object

   Instruction:
     - make sure the esmini shared library (esminiLib.dll or esminiLib.so) is present in esmini/bin folder
     - if not, either compile esmini (see User guide) or fetch bin package release
     - from this folder (where this code module is), run: ./change_speed.py (or python ./change_speed.py)
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

se.SE_ReportObjectSpeed.argtypes = [
    ctypes.c_int,
    ctypes.c_float
]

if se.SE_Init(b"../../../resources/xosc/cut-in.xosc", 0, 1, 0, 0) != 0:
    exit(-1)

state = 0
while se.SE_GetQuitFlag() != 1:

    if state == 0 and se.SE_GetSimulationTime() > 2:
        se.SE_ReportObjectSpeed(se.SE_GetId(0), 20.0)
        state = 1
    elif state == 1 and se.SE_GetSimulationTime() > 4:
        se.SE_ReportObjectSpeed(se.SE_GetId(0), 50.0)
        state = 1

    se.SE_Step()

se.SE_Close()