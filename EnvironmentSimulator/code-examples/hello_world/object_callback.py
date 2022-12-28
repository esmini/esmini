import ctypes as ct
from sys import platform

if platform == "linux" or platform == "linux2":
    se = ct.CDLL("../bin/libesminiLib.so")
elif platform == "darwin":
    se = ct.CDLL("../bin/libesminiLib.dylib")
elif platform == "win32":
    se = ct.CDLL("../bin/esminiLib.dll")
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
        ("timestamp", ct.c_float),
        ("x", ct.c_float),
        ("y", ct.c_float),
        ("z", ct.c_float),
        ("h", ct.c_float),
        ("p", ct.c_float),
        ("r", ct.c_float),
        ("roadId", ct.c_int),
        ("junctionId", ct.c_int),
        ("t", ct.c_float),
        ("laneId", ct.c_int),
        ("laneOffset", ct.c_float),
        ("s", ct.c_float),
        ("speed", ct.c_float),
        ("centerOffsetX", ct.c_float),
        ("centerOffsetY", ct.c_float),
        ("centerOffsetZ", ct.c_float),
        ("width", ct.c_float),
        ("length", ct.c_float),
        ("height", ct.c_float),
        ("objectType", ct.c_int),
        ("objectCategory", ct.c_int),
        ("wheel_angle", ct.c_float),
        ("wheel_rot", ct.c_float),
    ]


# Define callback for scenario object enabling manipulating the state AFTER scenario step but BEFORE OSI output
# Use in combination with ExternalController in mode=additive in order for scenario actions to be applied first
def callback(state_ptr, b):
    state = state_ptr.contents
    print("callback for obj {}: x={:.2f} y={:.2f} h={:.2f}".format(state.id, state.x, state.y, state.h))
    se.SE_ReportObjectPosXYH(ct.c_int(state.id), ct.c_float(state.timestamp), ct.c_float(state.x + 0.02), ct.c_float(state.y), ct.c_float(state.h + 0.001))

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
