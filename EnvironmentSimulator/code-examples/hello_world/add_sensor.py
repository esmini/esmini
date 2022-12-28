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

# specify some function return and argument types (needed for the floats)
se.SE_GetSimulationTime.restype = ctypes.c_float

se.SE_AddObjectSensor.argtypes = [
    ctypes.c_int,
    ctypes.c_float,
    ctypes.c_float,
    ctypes.c_float,
    ctypes.c_float,
    ctypes.c_float,
    ctypes.c_float,
    ctypes.c_float,
    ctypes.c_int
]

# Initialize scenario. Enable viewer and disable controllers.
se.SE_Init(b"../resources/xosc/synchronize.xosc", 1, 1, 0, 0)

# setup sensor stuff
max_detections = 10

# add one sensor to first vehicle (id 0)
if se.SE_AddObjectSensor(0, 4.0, 0.0, 0.5, 0.0, 1.0, 50.0, 60.0 * 3.1416 / 180.0, max_detections) != 0:
    print("Failed add sensor")

# show sensor frustum
se.SE_ViewerShowFeature(1, True);


# main loop
while se.SE_GetQuitFlag() != 1 and se.SE_GetSimulationTime() < 30.0:
    se.SE_Step()

    print('Detected object ids:', end='')
    obj_list = (ctypes.c_int * max_detections)()
    n_detections = se.SE_FetchSensorObjectList(0, obj_list)
    for i in range(n_detections):
        print(' {} '.format(obj_list[i]), end='')
    print('')

se.SE_Close()