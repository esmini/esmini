'''
   This script shows how to use the esmini UDPDriverModel, sending
   a few messages of different input modes for two objects.
   See testUDPDriverModel.py for a more comprehensive example.

   To run it:
   1. Open two terminals
   2. From terminal 1, run: ./bin/esmini --window 60 60 800 400 --osc ./scripts/udp-driver-model/two_cars_in_open_space.xosc
   3. From terminal 2, run: ./scripts/udp-driver-model/testUDPDriverModel-minimalistic-example.py --id 0 --id 1
        or python ./scripts/udp-driver-model/testUDPDriverModel-minimalistic-example.py --id 0 --id 1
        or python3 ./scripts/udp-driver-model/testUDPDriverModel-minimalistic-example.py --id 0 --id 1
        depending on platform and file type associations

   For complete message definitions, see esmini/EnvironmentSimulator/Modules/Controllers/ControllerUDPDriverModel.hpp
'''


import sys
import os
from socket import *
import struct
import time

input_modes = {
  'driverInput': 1,
  'stateXYZHPR': 2,
  'stateXYH': 3,
}

base_port = 49950

class UdpSender():
    def __init__(self, ip='127.0.0.1', port=base_port):
        # Create a UDP socket
        self.sock = socket(AF_INET, SOCK_DGRAM)
        self.addr = (ip, port)

    def send(self, msg):
        self.sock.sendto(msg, self.addr)

udpSender0 = UdpSender(port = base_port + 0)
udpSender1 = UdpSender(port = base_port + 1)

# Send a few frames

udpSender0.send(
    struct.pack(
        'iiiiddddd', 
        1,    # version
        input_modes['stateXYH'],
        0,    # object ID
        0,    # frame nr
        1.8,  # x
        6.5,  # y
        1.57, # h
        0,    # speed
        -0.9  # steering angle
    )
)

udpSender1.send(
    struct.pack(
        'iiiiddddd', 
        1,    # version
        input_modes['stateXYH'],
        1,    # object ID
        0,    # frame nr
        8.7,  # x
        6.5,  # y
        1.57, # h
        0.0,  # speed
        0.0   # steering angle
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
        0.25 # steering angle
    )
)

time.sleep(9)

udpSender1.send(
    struct.pack(
        'iiiiddd', 
        1,   # version
        input_modes['driverInput'],
        1,   # object ID
        0,   # frame nr
        0.0, # throttle
        0.1, # brake
        -0.2  # steering angle
    )
)

