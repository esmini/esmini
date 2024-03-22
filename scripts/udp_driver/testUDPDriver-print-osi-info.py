'''
   This script shows how to fetch and parse OSI message on UDP socket from esmini
   Prerequisites:
      Python 3

   Python dependencies:
      pip install protobuf==3.20.2

   To run it:
   1. Open two terminals
   2. From terminal 1, run: ./scripts/udp_driver/testUDPDriver-print-osi-info.py
        or python ./scripts/udp_driver/testUDPDriver-print-osi-info.py
        or python3 ./scripts/udp_driver/testUDPDriver-print-osi-info.py
        depending on platform and file type associations
   3. From terminal 2, run: ./bin/esmini --window 60 60 800 400 --osc ./scripts/udp_driver/one_car_on_road.xosc --osi_receiver_ip 127.0.0.1

   Note: The example involves printing of some static data, e.g. stationary objects, that is only posted in the first OSI frame. Hence,
         make sure to start the script BEFORE esmini, in order to not miss that very first OSI message.

   For complete driver control definitions, see esmini/EnvironmentSimulator/Modules/Controllers/ControllerUDPDriver.hpp
'''

from udp_osi_common import *

def print_osi_stuff(msg):

    print("OSI message timestamp: {:.2f} seconds".format(msg.timestamp.seconds + msg.timestamp.nanos * 1e-9))

    # Print some static content typically only available in first message
    print("{} lanes".format(len(msg.lane)))
    for i, l in enumerate(msg.lane):
        clf = l.classification
        print("  [{}] id {} type: {}".format(i, l.id.value, clf.type))
        print("    centerline:")
        for c_line in clf.centerline:
            print("    x: {:.2f} y: {:.2f}".format(c_line.x, c_line.y))

    print('{} stationary objects'.format(len(msg.stationary_object)))
    for i, s in enumerate(msg.stationary_object):
        print('  [{}] id {} type {}'.format(i, s.id.value, s.classification.type))
        print('    pos.x {:.2f} pos.y {:.2f} rot.h {:.2f}'.format(s.base.position.x, s.base.position.y, s.base.orientation.yaw))

    # Print some dynamic content from the message
    print('{} moving objects'.format(len(msg.moving_object)))
    for i, o in enumerate(msg.moving_object):
        print('  [{}] id {}'.format(i, o.id.value))
        print('    pos.x {:.2f} pos.y {:.2f} rot.h {:.2f}'.format(o.base.position.x, o.base.position.y, o.base.orientation.yaw))
        print('    vel.x {:.2f} vel.y {:.2f} rot_rate.h {:.2f}'.format(o.base.velocity.x, o.base.velocity.y, o.base.orientation_rate.yaw))
        print('    acc.x {:.2f} acc.y {:.2f} rot_acc.h {:.2f}'.format(o.base.acceleration.x, o.base.acceleration.y, o.base.orientation_acceleration.yaw))

        lane_id = o.assigned_lane_id[0].value if len(msg.lane) > 0 and len(o.assigned_lane_id) > 0 else -1
        left_lane_id = -1
        right_lane_id = -1
        for l in msg.lane:
            if l.id.value == o.assigned_lane_id[0].value:
                left_lane_id = l.classification.left_adjacent_lane_id[0].value if len(l.classification.left_adjacent_lane_id) > 0 else -1
                right_lane_id = l.classification.right_adjacent_lane_id[0].value if len(l.classification.right_adjacent_lane_id) > 0 else -1
                break
        print('    lane id {} left adj lane id {} right adj lane id {}'.format(lane_id, left_lane_id, right_lane_id))


if __name__ == "__main__":

    # Create UDP socket objects
    udpSender0 = UdpSender(port = base_port + 0)
    osiReceiver = OSIReceiver()
    done = False
    counter = 0

    # Press throttle to achieve some speed
    udpSender0.send(
        struct.pack(
            'iiiiddd',
            1,    # version
            input_modes['driverInput'],
            0,    # object ID
            0,    # frame nr
            0.1,  # throttle
            0.0,  # brake
            0.0   # steering angle
        )
    )

    while not done and counter < 10:
        # Read OSI
        try:
            msg = osiReceiver.receive()
            print_osi_stuff(msg)
        except timeout:
            print('osiReceive Timeout')
            done = True
        except KeyboardInterrupt:
            print('Ctrl+C pressed, quit')
            done = True

        counter += 1

    # Close and quit
    udpSender0.close()
    osiReceiver.close()
