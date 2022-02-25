# similar example as Python variant
# run from this folder (esmini/EnvironmentSimulator/code-examples/ad_hoc_traffic)

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

# Need to specify some function return and argument types
se.SE_GetSimulationTime.restype = ctypes.c_float
se.SE_ReportObjectPos.argtypes = [
        ctypes.c_int,
        ctypes.c_float,
        ctypes.c_float,
        ctypes.c_float,
        ctypes.c_float,
        ctypes.c_float,
        ctypes.c_float,
        ctypes.c_float,
        ctypes.c_float
    ]

# Define class Car to hold id and current position
class Car():
    def __init__(self, id, x_pos):
        self.id = id
        self.x_pos = x_pos

se.SE_AddPath(b"../../../resources")
se.SE_Init(b"../../../EnvironmentSimulator/code-examples/ad_hoc_traffic/empty_scenario.xosc", 1, 1, 0, 0)

cars = []  # list of cars
speed = 200.0 / 3.6
distance = 40.0

counter = 0
timestamp_now = 0.0
timestamp_now = se.SE_GetSimulationTime()
timestamp_old = timestamp_now

while (timestamp_now < 30.0 and not se.SE_GetQuitFlag() == 1):
    if timestamp_now > distance * counter / speed:
        # Add a vehicle at regular distance
        name = "object_" + str(counter)
        car_id = se.SE_AddObject(b"name", 1, 0, counter % 11)
        if car_id >= 0:
            cars.append(Car(car_id, 0.0))
        else:
            print("Failed to add car #{}", counter)
        counter += 1

    i = 0
    while i < len(cars):
        cars[i].x_pos += speed * (timestamp_now - timestamp_old)
        if cars[i].x_pos > 500:  # end of road
            print("Removing car with id {}", cars[i].id)
            se.SE_DeleteObject(cars[i].id)
            del cars[i]
            i -= 1
        else:
            se.SE_ReportObjectPos(cars[i].id, 0.0, cars[i].x_pos, -1.5, 0.0, 0.0, 0.0, 0.0, speed)
        i += 1
    
    se.SE_Step()
    timestamp_old = timestamp_now
    timestamp_now = se.SE_GetSimulationTime()
    if counter == 4:
        se.SE_SetCameraObjectFocus(3)
