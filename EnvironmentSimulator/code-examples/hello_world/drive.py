'''
    Demonstrate use of the simple vehicle model with interactive driving control

    Python dependencies:
        pip install keyboard

    Run from esmini/Hello-World_coding-example or esmini/bin:
        python ../EnvironmentSimulator/code-examples/hello_world/drive.py

    Drive with arrow keys
    Tune behavior with SE_SimpleVehicle* methods, e.g. SE_SimpleVehicleSteeringScale()
'''

import ctypes as ct
import keyboard
from keyboard._keyboard_event import KEY_DOWN, KEY_UP
from enum import IntEnum


esmini_path = '../'
se = ct.CDLL(esmini_path + "/bin/esminiLib.dll")

# specify some arguments and return types of useful functions
se.SE_SimpleVehicleCreate.argtypes = [ct.c_float, ct.c_float, ct.c_float, ct.c_float, ct.c_float]
se.SE_SimpleVehicleCreate.restype = ct.c_void_p
se.SE_SimpleVehicleGetState.argtypes = [ct.c_void_p, ct.c_void_p]
se.SE_SimpleVehicleControlBinary.argtypes = [ct.c_void_p, ct.c_double, ct.c_int, ct.c_int]
se.SE_ReportObjectWheelStatus.argtypes = [ct.c_int, ct.c_float, ct.c_float]
se.SE_ReportObjectPosXYH.argtypes = [ct.c_int, ct.c_float, ct.c_float, ct.c_float, ct.c_float]
se.SE_GetSimTimeStep.restype = ct.c_float
se.SE_StepDT.argtypes = [ct.c_float]

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
        ("wheelAngle", ct.c_float),
        ("wheelRot", ct.c_float),
    ]

# Definition of SE_SimpleVehicleState struct
class SESimpleVehicleState(ct.Structure):
    _fields_ = [
         ("x", ct.c_float),
         ("y", ct.c_float),
         ("z", ct.c_float),
         ("h", ct.c_float),
         ("p", ct.c_float),
         ("speed", ct.c_float),
         ("wheel_rotation", ct.c_float),
         ("wheel_angle", ct.c_float),
    ]

class Vehicle:
    def __init__(self, x, y, h):
        self.wheel = 0 # -1 for left turn, 0 for no turn, 1 for right turn
        self.pedal = 0 # -1 for braking, 0 for no acceleration, 1 for accelerating
        self.sv_handle = se.SE_SimpleVehicleCreate(x, y, h, 5.0, 0.0)
        self.vh_state = SESimpleVehicleState()  # used to fetch the state of the vehicle model object

    def update(self, dt: float):
        se.SE_SimpleVehicleControlBinary(self.sv_handle, dt, self.pedal, self.wheel)
        se.SE_SimpleVehicleGetState(self.sv_handle, ct.byref(self.vh_state))

# setup keyboard callback
class ArrowKey(IntEnum):
    UP    = 72
    DOWN  = 80
    RIGHT = 77
    LEFT  = 75

def on_action(event):
    if event.event_type == KEY_DOWN:
        if event.scan_code == ArrowKey.UP:
            vehicle.pedal = 1
        elif event.scan_code == ArrowKey.DOWN:
            vehicle.pedal = -1
        elif event.scan_code == ArrowKey.RIGHT:
            vehicle.wheel = -1
        elif event.scan_code == ArrowKey.LEFT:
            vehicle.wheel = 1
    elif event.event_type == KEY_UP:
        if event.scan_code == ArrowKey.UP or event.scan_code == ArrowKey.DOWN:
            vehicle.pedal = 0
        if event.scan_code == ArrowKey.RIGHT or event.scan_code == ArrowKey.LEFT:
            vehicle.wheel = 0

keyboard.hook(lambda e: on_action(e))

# initialize esmini
se.SE_AddPath(esmini_path.encode() + b"resources/models")
if se.SE_Init(esmini_path.encode() + b"EnvironmentSimulator/Unittest/xosc/dummy_mw.xosc", 0, 1, 0, 0) != 0:
    print('Failed to initialize esmini')
    exit(-1)

# fetch initial position from the scenario
obj_state = SEScenarioObjectState()
se.SE_GetObjectState(se.SE_GetId(0), ct.byref(obj_state))

# create vehicle at initial pose
vehicle = Vehicle(obj_state.x, obj_state.y, obj_state.h)

while se.SE_GetQuitFlag() == 0:
    dt = se.SE_GetSimTimeStep()
    vehicle.update(dt)
    se.SE_ReportObjectPosXYH(se.SE_GetId(0), 0.0, vehicle.vh_state.x, ct.c_float(vehicle.vh_state.y), vehicle.vh_state.h)
    se.SE_ReportObjectWheelStatus(0, vehicle.vh_state.wheel_rotation, vehicle.vh_state.wheel_angle)
    se.SE_Step()
