'''
   This script shows how to use the esmini UDPDriverController in combination with
   OSI. Two driver control messages is sent, one for each vehicle. Then one OSI message
   is fetched and parsed.
   Prerequisites:
      Python 3

   Python dependencies:
      pip install protobuf==3.20.2

   To run it:
   1. Open two terminals
   2. From terminal 1, run: ./bin/esmini --window 60 60 800 400 --osc ./scripts/udp_driver/two_cars_in_open_space.xosc --osi_receiver_ip 127.0.0.1
   3. From terminal 2, run: ./scripts/udp_driver/testUDPDriver-minimalistic-example-osi.py --id 0 --id 1
        or python ./scripts/udp_driver/testUDPDriver-minimalistic-example-osi.py --id 0 --id 1
        or python3 ./scripts/udp_driver/testUDPDriver-minimalistic-example-osi.py --id 0 --id 1
        depending on platform and file type associations

   For complete driver control message definitions, see esmini/EnvironmentSimulator/Modules/Controllers/ControllerUDPDriver.hpp
'''

from udp_osi_common import *

if __name__ == "__main__":

    # Create UDP socket objects
    udpSender0 = UdpSender(port = base_port + 0)
    udpSender1 = UdpSender(port = base_port + 1)
    osiReceiver = OSIReceiver()

    # Send states
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
            5.0,  # speed
            0.0,  # steering angle
            1     # dead reckoning
        )
    )

    # Expect OSI message
    try:
        msg = osiReceiver.receive()

        # Print some content from the message
        for i, o in enumerate(msg.moving_object):
            print('Object[{}] id {}'.format(i, o.id.value))
            print('   pos.x {:.2f} pos.y {:.2f} rot.h {:.2f}'.format(o.base.position.x, o.base.position.y, o.base.orientation.yaw))
            print('   vel.x {:.2f} vel.y {:.2f} rot_rate.h {:.2f}'.format(o.base.velocity.x, o.base.velocity.y, o.base.orientation_rate.yaw))
            print('   acc.x {:.2f} acc.y {:.2f} rot_acc.h {:.2f}'.format(o.base.acceleration.x, o.base.acceleration.y, o.base.orientation_acceleration.yaw))
    except timeout:
        print('caught a timeout')

    # Close and quit
    udpSender0.close()
    udpSender1.close()
    osiReceiver.close()
