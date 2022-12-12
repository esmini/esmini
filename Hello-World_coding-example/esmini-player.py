import ctypes
import sys

if sys.platform == "linux" or sys.platform == "linux2":
    se = ctypes.CDLL("../bin/libesminiLib.so")
elif sys.platform == "darwin":
    se = ctypes.CDLL("../bin/libesminiLib.dylib")
elif sys.platform == "win32":
    se = ctypes.CDLL("../bin/esminiLib.dll")
else:
    print("Unsupported platform: {}".format(sys.platform))
    quit()

if (len(sys.argv) < 2):
    print('Usage: {} <xosc file>'.format(sys.argv[0]))
    exit(-1)

se.SE_Init(sys.argv[1].encode('ascii'), 0, 1, 0, 0)

while not se.SE_GetQuitFlag():
    se.SE_Step()
