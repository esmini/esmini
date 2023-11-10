# initialize esmini with arguments instead of a few limited arguments
# using ideas from: https://comp.lang.python.narkive.com/RUdYlz64/trying-to-pass-sys-argv-as-int-argc-char-argv-using-ctypes

import ctypes as ct
import sys

if sys.platform == "linux" or sys.platform == "linux2":
    se = ct.CDLL("../bin/libesminiLib.so")
elif sys.platform == "darwin":
    se = ct.CDLL("../bin/libesminiLib.dylib")
elif sys.platform == "win32":
    se = ct.CDLL("../bin/esminiLib.dll")
else:
    print("Unsupported platform: {}".format(platform))
    quit()

# specify arguments types of esmini function
se.SE_InitWithArgs.argtypes = [ct.c_int, ct.POINTER(ct.POINTER(ct.c_char))]

# create the list of arguments
args = [
    "--window", "60", "60", "800", "400",
    "--osc", "../resources/xosc/cut-in.xosc",
    ]

# prepare argument list for ctypes use
argv = (ct.POINTER(ct.c_char) * len(args))()
argc = len(argv)
for i, arg in enumerate(args):
    argv[i] = ct.create_string_buffer(arg.encode())

# init esmini
if se.SE_InitWithArgs(argc, argv) != 0:
    exit(-1)

# execute esmini until end of scenario or user requested quit by ESC key
while se.SE_GetQuitFlag() == 0:
    se.SE_Step()
