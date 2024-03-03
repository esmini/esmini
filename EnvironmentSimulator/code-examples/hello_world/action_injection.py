import ctypes as ct
import sys

if sys.platform == "linux" or sys.platform == "linux2":
    se = ct.CDLL("../bin/libesminiLib.so")
elif sys.platform == "darwin":
    se = ct.CDLL("../bin/libesminiLib.dylib")
elif sys.platform == "win32":
    se = ct.CDLL("../bin/esminiLib.dll")
else:
    print("Unsupported platform: {}".format(sys.platform))
    quit()

if (len(sys.argv) < 2):
    print('Usage: {} <xosc file>'.format(sys.argv[0]))
    exit(-1)

# Definition of SE_ScenarioObjectState struct
class SESpeedActionStruct(ct.Structure):
    _fields_ = [
        ("id", ct.c_int),                # id of object to perform action
        ("speed", ct.c_float),
        ("transition_shape", ct.c_int),  # 0 = cubic, 1 = linear, 2 = sinusoidal, 3 = step
        ("transition_dim", ct.c_int),    # 0 = distance, 1 = rate, 2 = time
        ("transition_value", ct.c_float),
    ]

class SELaneChangeActionStruct(ct.Structure):
    _fields_ = [
        ("id", ct.c_int),                # id of object to perform action
        ("mode", ct.c_int),              # 0 = absolute, 1 = relative (own vehicle)
        ("target", ct.c_int),            # target lane id (absolute or relative)
        ("transition_shape", ct.c_int),  # 0 = cubic, 1 = linear, 2 = sinusoidal, 3 = step
        ("transition_dim", ct.c_int),    # 0 = distance, 1 = rate, 2 = time
        ("transition_value", ct.c_float),
    ]

class SELaneOffsetActionStruct(ct.Structure):
    _fields_ = [
        ("id", ct.c_int),                # id of object to perform action
        ("offset", ct.c_float),
        ("max_lateral_acc", ct.c_float),
        ("transition_shape", ct.c_int),  # 0 = cubic, 1 = linear, 2 = sinusoidal, 3 = step
    ]


# specify some arguments and return types of useful functions
se.SE_StepDT.argtypes = [ct.c_float]
se.SE_GetSimulationTime.restype = ct.c_float

# initialize some structs needed for actions
lane_offset_action = SELaneOffsetActionStruct()
lane_change_action = SELaneChangeActionStruct()
speed_action = SESpeedActionStruct()

# initialize esmini with provided scenario
se.SE_Init(sys.argv[1].encode('ascii'), 0, 1, 0, 0)

state = 0  # minimalistic state machine for trigging various actoins

while se.SE_GetQuitFlag() == 0 and se.SE_GetSimulationTime() < 20.0:

    if state == 0 and se.SE_GetSimulationTime() > 2.0:
        print("Injecting lane offset action");
        lane_offset_action.id               = 0;
        lane_offset_action.offset           = -0.45;
        lane_offset_action.max_lateral_acc  = 0.5;
        lane_offset_action.transition_shape = 0;
        se.SE_InjectLaneOffsetAction(ct.byref(lane_offset_action));
        state += 1
    elif state == 1 and se.SE_GetSimulationTime() > 7.0:
        print("Injecting lane change action");
        lane_change_action.id               = 0;
        lane_change_action.mode             = 1;
        lane_change_action.target           = 1;
        lane_change_action.transition_shape = 2;
        lane_change_action.transition_dim   = 2;
        lane_change_action.transition_value = 3.0;
        se.SE_InjectLaneChangeAction(ct.byref(lane_change_action));
        state += 1
    elif state == 2 and se.SE_GetSimulationTime() > 9.5:
        print("Injecting speed action");
        speed_action.id               = 0;
        speed_action.speed            = 0.0;
        speed_action.transition_shape = 0;
        speed_action.transition_dim   = 1;
        speed_action.transition_value = 5.0;
        se.SE_InjectSpeedAction(ct.byref(speed_action));
        state += 1

    # step the simulation in natural speed, change to SE_Step(<time-step>) for fixed timestep
    se.SE_Step()
