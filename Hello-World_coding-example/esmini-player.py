import ctypes
from sys import platform

if platform == "linux" or platform == "linux2":
    se = ctypes.CDLL("./libesminiLib.so")
elif platform == "darwin":
    se = ctypes.CDLL("./libesminiLib.dylib")
elif platform == "win32":
    se = ctypes.CDLL("./esminiLib.dll")
else:
    print("Unsupported platform: {}".format(platform))
    quit()
    
se.SE_Init(b"../resources/xosc/cut-in.xosc", 0, 1, 0, 0)
 
for i in range (500):
    se.SE_Step()
