import ctypes as ct
import sys

if sys.platform == "linux" or sys.platform == "linux2":
    rm = ct.CDLL("../bin/libesminiRMLib.so")
elif sys.platform == "darwin":
    rm = ct.CDLL("../bin/libesminiRMLib.dylib")
elif sys.platform == "win32":
    rm = ct.CDLL("../../../bin/esminiRMLib.dll")
else:
    print("Unsupported platform: {}".format(sys.platform))
    sys.exit(-1)



class RM_PositionXYZ(ct.Structure):
    _fields_ = [
        ("x", ct.c_float),
        ("y", ct.c_float),
        ("z", ct.c_float),
    ]


class RM_RoadLaneInfo(ct.Structure):
    _fields_ = [
        ("pos", RM_PositionXYZ),
        ("heading", ct.c_float),
        ("pitch", ct.c_float),
        ("roll", ct.c_float),
        ("width", ct.c_float),
        ("curvature", ct.c_float),
        ("speed_limit", ct.c_float),
        ("roadId", ct.c_int),
        ("junctionId", ct.c_int),  # -1 if not in a junction
        ("laneId", ct.c_int),
        ("lane_offset", ct.c_float),
        ("s", ct.c_float),
        ("t", ct.c_float),
    ]


# Specify argument types to a few functions
rm.RM_SetWorldPosition.argtypes = [ct.c_int, ct.c_float, ct.c_float, ct.c_float, ct.c_float, ct.c_float, ct.c_float]
rm.RM_GetLaneInfo.argtypes = [ct.c_int, ct.c_float, ct.POINTER(RM_RoadLaneInfo), ct.c_int, ct.c_int]

# Initialize esmini RoadManger with given OpenDRIVE file
odr = '../../../resources/xodr/straight_500m.xodr'
if rm.RM_Init(odr.encode()) == -1:   # encode() converts string to pure byte array
    print("Failed to load OpenDRIVE file ", )
    sys.exit(-1)

rm_pos = rm.RM_CreatePosition()         # create a position object, returns a handle
rm_road_lane_info = RM_RoadLaneInfo()   # object that will be passed and filled in with road lane info

x = -20.75
y = -137
rm.RM_SetWorldPosition(rm_pos, x, y, 0.0, 0.0, 0.0, 0.0)
rm.RM_GetLaneInfo(rm_pos, 0.0, ct.byref(rm_road_lane_info), 2, False)

print('road_id {} curvature {} '.format(rm_road_lane_info.roadId, rm_road_lane_info.curvature))

