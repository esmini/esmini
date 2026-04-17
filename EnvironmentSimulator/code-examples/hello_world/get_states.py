'''
   demonstrate how to get scenario object states

   Instruction:
     - make sure the esmini shared library (esminiLib.dll or esminiLib.so) is present in esmini/bin folder
     - if not, either compile esmini (see User guide) or fetch bin package release
     - from this folder (where this code module is), run: python ./get_states.py
'''

import ctypes
from sys import platform

if platform == "linux" or platform == "linux2":
    se = ctypes.CDLL("../../../bin/libesminiLib.so")
elif platform == "darwin":
    se = ctypes.CDLL("../../../bin/libesminiLib.dylib")
elif platform == "win32":
    se = ctypes.CDLL("../../../bin/esminiLib.dll")
else:
    print("Unsupported platform: {}".format(platform))
    quit()

# Definition of SE_ScenarioObjectState struct
class SEScenarioObjectState(ctypes.Structure):
    _fields_ = [
        ("id", ctypes.c_int),
        ("model_id", ctypes.c_int),
        ("control", ctypes.c_int),
        ("timestamp", ctypes.c_double),
        ("x", ctypes.c_double),
        ("y", ctypes.c_double),
        ("z", ctypes.c_double),
        ("h", ctypes.c_double),
        ("p", ctypes.c_double),
        ("r", ctypes.c_double),
        ("roadId", ctypes.c_int),
        ("junctionId", ctypes.c_int),
        ("t", ctypes.c_double),
        ("laneId", ctypes.c_int),
        ("laneOffset", ctypes.c_double),
        ("s", ctypes.c_double),
        ("speed", ctypes.c_double),
        ("centerOffsetX", ctypes.c_double),
        ("centerOffsetY", ctypes.c_double),
        ("centerOffsetZ", ctypes.c_double),
        ("width", ctypes.c_double),
        ("length", ctypes.c_double),
        ("height", ctypes.c_double),
        ("objectType", ctypes.c_int),
        ("objectCategory", ctypes.c_int),
        ("wheelAngle", ctypes.c_double),
        ("wheelRot", ctypes.c_double),
        ("visibilityMask", ctypes.c_int)
    ]

se.SE_Init(b"../resources/xosc/cut-in.xosc", 0, 1, 0, 0)

obj_state = SEScenarioObjectState()  # object that will be passed and filled in with object state info

for i in range(500):
    for j in range(se.SE_GetNumberOfObjects()):
        se.SE_GetObjectState(se.SE_GetId(j), ctypes.byref(obj_state))
        print('Frame {} Time {:.2f} ObjId {} roadId {} laneId {} laneOffset {:.2f} s {:.2f} x {:.2f} y {:.2f} heading {:.2f} speed {:.2f} wheelAngle {:.2f} wheelRot {:.2f}'.format(
            i, obj_state.timestamp, obj_state.id, obj_state.roadId, obj_state.laneId, obj_state.laneOffset,
            obj_state.s, obj_state.x, obj_state.y, obj_state.h, obj_state.speed * 3.6, obj_state.wheelAngle, obj_state.wheelRot))
    se.SE_Step()
