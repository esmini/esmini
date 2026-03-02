'''
   demonstrate how to use the object callback mechanism in Python context
   using ideas from: https://comp.lang.python.narkive.com/RUdYlz64/trying-to-pass-sys-argv-as-int-argc-char-argv-using-ctypes

   Instruction:
     - make sure the esmini shared library (esminiLib.dll or esminiLib.so) is present in esmini/bin folder
     - if not, either compile esmini (see User guide) or fetch bin package release
     - from this folder (where this code module is), run: python ./object_callback.py
'''

import ctypes as ct
from sys import platform

if platform == "linux" or platform == "linux2":
    se = ct.CDLL("../../../bin/libesminiLib.so")
elif platform == "darwin":
    se = ct.CDLL("../../../bin/libesminiLib.dylib")
elif platform == "win32":
    se = ct.CDLL("../../../bin/esminiLib.dll")
else:
    print("Unsupported platform: {}".format(platform))
    quit()

# asciidoc tag::ObjectCallback[]

# Definition of SE_ScenarioObjectState struct
class SEScenarioObjectState(ct.Structure):
    _fields_ = [
        ("id", ct.c_int),
        ("model_id", ct.c_int),
        ("control", ct.c_int),
        ("timestamp", ct.c_double),
        ("x", ct.c_double),
        ("y", ct.c_double),
        ("z", ct.c_double),
        ("h", ct.c_double),
        ("p", ct.c_double),
        ("r", ct.c_double),
        ("roadId", ct.c_int),
        ("junctionId", ct.c_int),
        ("t", ct.c_double),
        ("laneId", ct.c_int),
        ("laneOffset", ct.c_double),
        ("s", ct.c_double),
        ("speed", ct.c_double),
        ("centerOffsetX", ct.c_double),
        ("centerOffsetY", ct.c_double),
        ("centerOffsetZ", ct.c_double),
        ("width", ct.c_double),
        ("length", ct.c_double),
        ("height", ct.c_double),
        ("objectType", ct.c_int),
        ("objectCategory", ct.c_int),
        ("wheel_angle", ct.c_double),
        ("wheel_rot", ct.c_double),
    ]


# Define callback for scenario object enabling manipulating the state AFTER scenario step but BEFORE OSI output
# Use in combination with ExternalController in mode=additive in order for scenario actions to be applied first
def callback(state_ptr, b):
    state = state_ptr.contents
    print("callback for obj {}: x={:.2f} y={:.2f} h={:.2f}".format(state.id, state.x, state.y, state.h))
    se.SE_ReportObjectPosXYH(ct.c_int(state.id), ct.c_double(state.x + 0.02), ct.c_double(state.y), ct.c_double(state.h + 0.001))

callback_type = ct.CFUNCTYPE(None, ct.POINTER(SEScenarioObjectState), ct.c_void_p)
callback_func = callback_type(callback)

# Initialize esmini before register the callback
se.SE_AddPath(b"../../../resources/xosc")  # search path if run from original location
se.SE_Init(b"../resources/xosc/cut-in.xosc", 0, 1, 0, 1)

# register callback for first object (id=0)
se.SE_RegisterObjectCallback(0, callback_func, 0)

# asciidoc end::ObjectCallback[]

for i in range(500):
    se.SE_Step()
