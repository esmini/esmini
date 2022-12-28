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

# Specify argument types to a few functions
rm.RM_SetWorldPosition.argtypes = [ct.c_int, ct.c_float, ct.c_float, ct.c_float, ct.c_float, ct.c_float, ct.c_float]
rm.RM_SetLanePosition.argtypes = [ct.c_int, ct.c_int, ct.c_int, ct.c_float, ct.c_float]


# Initialize esmini RoadManger with given OpenDRIVE file
odr = '../resources/xodr/straight_500m.xodr'
if rm.RM_Init(odr.encode()) == -1:   # encode() converts string to pure byte array
    print("Failed to load OpenDRIVE file ", )
    sys.exit(-1)

rm_pos = rm.RM_CreatePosition()  # create a position object, returns a handle
rm_pos_data = RM_PositionData()  # object that will be passed and filled in with position info

# test a few positions
x = 65
y = -1.7
rm.RM_SetWorldPosition(rm_pos, x, y, 0.0, 0.0, 0.0, 0.0)
rm.RM_GetPositionData(rm_pos, ct.byref(rm_pos_data))
print('road_id {} lane_id {} lane_offset {:.2f} s {:.2f}'.format(
    rm_pos_data.road_id, rm_pos_data.lane_id, rm_pos_data.lane_offset, rm_pos_data.s))

road_id = 1
lane_id = 1
lane_offset = -0.2
s = 40
rm.RM_SetLanePosition(rm_pos, road_id, lane_id, lane_offset, s)
rm.RM_GetPositionData(rm_pos, ct.byref(rm_pos_data))
print('x {:.2f} y {:.2f} h (yaw) {:.2f}'.format(
    rm_pos_data.x, rm_pos_data.y, rm_pos_data.h))
