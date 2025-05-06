'''
   Python dependencies:
      pip install protobuf==3.20.2

   Note: No need to install OSI as long as
   esmini/scripts/osi3 folder is available
'''

import os
import sys
import ctypes as ct

# Add scripts root directory to module search path in order to find osi3
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../', 'scripts')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'scripts')))  # if script moved to Hello World folder

from osi3.osi_groundtruth_pb2 import GroundTruth

# Import esmini lib
if sys.platform == "linux" or sys.platform == "linux2":
    se = ct.CDLL("../bin/libesminiLib.so")
elif sys.platform == "darwin":
    se = ct.CDLL("../bin/libesminiLib.dylib")
elif sys.platform == "win32":
    se = ct.CDLL("../bin/esminiLib.dll")
else:
    print("Unsupported platform: {}".format(platform))
    quit()


# Specify argument and return types of some esmini functions
se.SE_GetOSIGroundTruth.restype = ct.c_void_p
se.SE_GetOSIGroundTruth.argtypes = [ct.c_void_p]
se.SE_StepDT.argtypes = [ct.c_float]

class Sim:
    def __init__(self, scenario):
        self.msg = GroundTruth()  # OSI groundtruth message
        self.msg_size = ct.c_int()
        if se.SE_Init(scenario, 0, 1, 0, 0) != 0:
            print('Failed to open ', scenario)
            exit(-1)

        self.GetAndPrintOSIInfo()  # Initial state

    # Retrieve OSI message and print some info
    def GetAndPrintOSIInfo(self):
        msg_string = se.SE_GetOSIGroundTruth(ct.byref(self.msg_size))
        self.msg.ParseFromString(ct.string_at(msg_string, self.msg_size))

        # Print some info
        print("Timestamp: {:.2f}".format(self.msg.timestamp.seconds + 1e-9 * self.msg.timestamp.nanos))
        for j, o in enumerate(self.msg.moving_object):
            print('   Object[{}] id {}'.format(j, o.id.value))
            print('      pos.x {:.2f} pos.y {:.2f} rot.h {:.2f}'.format(\
                o.base.position.x,\
                o.base.position.y,\
                o.base.orientation.yaw))
            print('      vel.x {:.2f} vel.y {:.2f} rot_rate.h {:.2f}'.format(\
                o.base.velocity.x,\
                o.base.velocity.y,\
                o.base.orientation_rate.yaw))
            print('      acc.x {:.2f} acc.y {:.2f} rot_acc.h {:.2f}'.format(\
                o.base.acceleration.x,\
                o.base.acceleration.y,\
                o.base.orientation_acceleration.yaw))


    def Run(self):
        while se.SE_GetQuitFlag() != 1:
            se.SE_StepDT(0.05)
            self.GetAndPrintOSIInfo()  # Updated state

        se.SE_Close()

if __name__ == '__main__':
    # Execute when the module is not initialized from an import statement.

    sim = Sim(b"../resources/xosc/cut-in.xosc")
    sim.Run()
