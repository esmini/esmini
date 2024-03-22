'''
   This script hints how to implement a driver model using the esmini UDPDriverController and OSI.
   The driver model will try to follow a pre-defined trajectory. It will set throttle, brake and steering angle 
   based on current position and orientation which is extracted from OSI UDP messages from esmini. 
   Prerequisites:
      Python 3

   Python dependencies:
      pip install protobuf==3.20.2

   To run it:
   1. Open two terminals
   2. From terminal 1, run: ./bin/esmini --window 60 60 800 400 --osc ./scripts/udp_driver/follow_trajectory.xosc --osi_receiver_ip 127.0.0.1
   3. From terminal 2, run: ./scripts/udp_driver/testUDPDriver-follow-trajectory.py
        or python ./scripts/udp_driver/testUDPDriver-follow-trajectory.py
        or python3 ./scripts/udp_driver/testUDPDriver-follow-trajectory.py
        depending on platform and file type associations
'''

import math
from operator import truediv
import time
import sys
from udp_osi_common import *

class Driver():
    def __init__(self):
        self.steering = 0.0
        self.speed = 0.0
        self.target_speed = 50 / 3.6  # km/h
        self.throttle = 0.0
        self.brake = 0.0
    
    def trajectory_function(self, x):
        # linear zigzag shape 
        # First a line from 0,0 to 100,100 (y=x)
        # Then a line from 100,100 to 200,0 (y=-x+200)
        # Then repeat infinitely along x. 
        if x % 200 < 100:
            return x % 200
        else:
            return -(x % 200) + 200

    def step(self, speed, x, y, h):

        lookahead = max(5.0, 1.2 * speed)  # look ahead some distance proportional to speed
        # Lateral position on the trajectory
        y_target = self.trajectory_function(x+lookahead)  # look ahead 10 m

        # Calculate angle to target point from current location
        angle = math.atan((y_target - y) / ((x+lookahead) - x))
        # Subtract current heading/yaw of the vehicle and apply some scaling factor
        self.steering = 0.5 * (angle - h)

        # Give throttle or brake to reach target_speed
        if self.target_speed - speed > 0:
            self.throttle =  0.05 * (self.target_speed - speed)
            self.brake = 0.0
        else:
            self.throttle =  0.0
            self.brake = 0.05 * (speed - self.target_speed)

        # slow down with increased steering angle
        self.throttle -= 0.4 * abs(self.steering)
        self.brake += 0.4 * abs(self.steering)

def print_osi_stuff(msg):
    # Print some content from the message
    print('lane ids: {}'.format([l.id.value for l in msg.lane]))
    for i, o in enumerate(msg.moving_object):
        print('Object[{}] id {}'.format(i, o.id.value))
        print('   pos.x {:.2f} pos.y {:.2f} rot.h {:.2f}'.format(o.base.position.x, o.base.position.y, o.base.orientation.yaw))
        print('   vel.x {:.2f} vel.y {:.2f} rot_rate.h {:.2f}'.format(o.base.velocity.x, o.base.velocity.y, o.base.orientation_rate.yaw))
        print('   acc.x {:.2f} acc.y {:.2f} rot_acc.h {:.2f}'.format(o.base.acceleration.x, o.base.acceleration.y, o.base.orientation_acceleration.yaw))
        
        lane_id = o.assigned_lane_id[0].value if len(msg.lane) > 0 and len(o.assigned_lane_id) > 0 else -1
        left_lane_id = -1
        right_lane_id = -1
        for l in msg.lane:
            if l.id.value == o.assigned_lane_id[0].value:
                left_lane_id = l.classification.left_adjacent_lane_id[0].value if len(l.classification.left_adjacent_lane_id) > 0 else -1
                right_lane_id = l.classification.right_adjacent_lane_id[0].value if len(l.classification.right_adjacent_lane_id) > 0 else -1
                break
        print('   lane id {} left adj lane id {} right adj lane id {}'.format(lane_id, left_lane_id, right_lane_id))


if __name__ == "__main__":

    id = 0

    # Create UDP socket objects
    udpSender0 = UdpSender(port = base_port + id)
    osiReceiver = OSIReceiver()
    driver = Driver() 
    done = False
    counter = 0

    while not done:
       
        # Read OSI 
        try:
            msg = osiReceiver.receive()
            print_osi_stuff(msg)

            # extract values for vehicle
            o = msg.moving_object[id]
            speed = math.sqrt(o.base.velocity.x**2 + o.base.velocity.y**2)  # assume speed is the hypotenuse of x and y velocity components
            driver.step(speed, o.base.position.x, o.base.position.y, o.base.orientation.yaw)
        
        except timeout:
            print('osiReceive Timeout')
        except KeyboardInterrupt:
            print('Ctrl+C pressed, quit')
            done = True

        # Send updated driver input
        udpSender0.send(
            struct.pack(
                'iiiiddd', 
                1,    # version
                input_modes['driverInput'],
                id,    # object ID
                counter,    # frame nr
                driver.throttle, # throttle
                driver.brake,  # brake
                driver.steering # steering angle
            )
        )

        print('{} throttle {:.2f} brake {:.2f} steer_angle {:.2f}'.format(counter, driver.throttle, driver.brake, driver.steering ))

        counter += 1

    # Close and quit
    udpSender0.close()
    osiReceiver.close()
