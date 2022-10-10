'''
   This script shows how to use the esmini UDPDriverController, sending
   a few messages of different input modes for two objects.
   See testUDPDriverModel.py for a more comprehensive example.
   Prerequisites:
      Python 3

   To run it:
   1. Open two terminals
   2. From terminal 1, run: ./bin/esmini --window 60 60 800 400 --osc ./scripts/udp_driver/two_cars_in_open_space.xosc
   3. From terminal 2, run: ./scripts/udp_driver/testUDPDriver-minimalistic-example.py --id 0 --id 1
        or python ./scripts/udp_driver/testUDPDriver-minimalistic-example.py --id 0 --id 1
        or python3 ./scripts/udp_driver/testUDPDriver-minimalistic-example.py --id 0 --id 1
        depending on platform and file type associations

   For complete message definitions, see esmini/EnvironmentSimulator/Modules/Controllers/ControllerUDPDriver.hpp
'''


from udp_osi_common import *
import time


udpSender0 = UdpSender(port = base_port + 0)
udpSender1 = UdpSender(port = base_port + 1)

# Send a few frames

udpSender0.send(
    struct.pack(
        'iiiidddddB',
        1,    # version
        input_modes['stateXYH'],
        0,    # object ID
        0,    # frame nr
        1.8,  # x
        6.5,  # y
        1.57, # h
        0,    # speed
        -0.9, # steering angle
        1     # dead reckoning
    )
)

udpSender1.send(
    struct.pack(
        'iiiidddddB',
        1,    # version
        input_modes['stateXYH'],
        1,    # object ID
        0,    # frame nr
        8.7,  # x
        6.5,  # y
        1.57, # h
        0.0,  # speed
        0.0,   # steering angle
        1     # dead reckoning
    )
)

time.sleep(3)

udpSender1.send(
    struct.pack(
        'iiiiddd',
        1,    # version
        input_modes['driverInput'],
        1,    # object ID
        0,    # frame nr
        0.08, # throttle
        0.0,  # brake
        0.6 # steering angle
    )
)

time.sleep(10)

udpSender1.send(
    struct.pack(
        'iiiiddd',
        1,   # version
        input_modes['driverInput'],
        1,   # object ID
        0,   # frame nr
        0.0, # throttle
        0.1, # brake
        -0.4  # steering angle
    )
)
