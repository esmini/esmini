'''
   This script shows how to reverse using the DRIVER_INPUT mode
   Prerequisites:
      Python 3

   To run it:
   1. Open two terminals
   2. From terminal 1, run: ./bin/esmini --window 60 60 800 400 --osc ./scripts/udp_driver/two_cars_in_open_space.xosc
   3. From terminal 2, run: ./scripts/udp_driver/testUDPDriver-forward-reverse.py --id 0
        or python ./scripts/udp_driver/testUDPDriver-forward-reverse.py --id 0
        or python3 ../scripts/udp_driver/testUDPDriver-forward-reverse.py --id 0
        depending on platform and file type associations

   For complete message definitions, see esmini/EnvironmentSimulator/Modules/Controllers/ControllerUDPDriver.hpp
'''


from udp_osi_common import *
import time


udpSender0 = UdpSender(port = base_port + 0)
frame_nr = 0

def send(throttle, brake):
    global frame_nr
    global udpSender0
    object_id = 0
    steering = 0.0

    udpSender0.send(
        struct.pack('iiiiddd',
                1,
                input_modes['driverInput'],
                object_id,
                frame_nr,
                throttle,
                brake,
                steering)
    )
    frame_nr += 1


print("accelerate")
send(0.5, 0.0)
time.sleep(3)

print("brake to stop")
send(0.0, 1.0)
time.sleep(3)

print("release pedals")
send(0.0, 0.0)
time.sleep(1)

print("reverse")
send(0.0, 0.5)
time.sleep(3)
