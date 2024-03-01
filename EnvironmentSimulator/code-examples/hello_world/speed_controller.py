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

# specify some arguments of useful functions
se.SE_ReportObjectSpeed.argtypes = [ctypes.c_int, ctypes.c_float]
se.SE_StepDT.argtypes = [ctypes.c_float]

dt    = 0.05  # fixed timestep
speed = 2.0   # initial speed
acc   = 10.0  # initial acceleration

# initialize esmini with provided scenario
se.SE_Init(sys.argv[1].encode('ascii'), 0, 1, 0, 0)

while not se.SE_GetQuitFlag():

    speed += acc * dt

    # modulate speed by changing sign of acceleration now and then
    if speed > 20.0 or speed < 2.0:
        acc = -acc

    # report new speed
    se.SE_ReportObjectSpeed(0, speed)

    # step simulation with fixed timestep  (use SE_Step() for realtime)
    se.SE_StepDT(dt)
