import os
import struct
import argparse
import math

from udp_driver.osi3.osi_groundtruth_pb2 import *


class OSIFile():
    def __init__(self, filename):
        if not os.path.isfile(filename):
            print('ERROR: OSI file not found: {}'.format(filename))
            return
        try:
            self.file = open(filename, 'rb')
        except OSError:
            print('ERROR: Could not open file {} for reading'.format(filename))
            raise

        self.filename = filename
        self.osi_msg = GroundTruth()

    def save_csv(self):
        csvfile = os.path.splitext(self.filename)[0] + '.csv'
        try:
            fcsv = open(csvfile, 'w')
        except OSError:
            print('ERROR: Could not open file {} for writing'.format(csvfile))
            raise

        # write header
        fcsv.write('time, id, name, x, y, z, vx, vy, vz, h, p, r, speed, wheel_angle, wheel_rot\n')

        # write data
        while self.read_next_message():
            t = self.osi_msg.timestamp.seconds + self.osi_msg.timestamp.nanos * 1e-9
            for o in self.osi_msg.moving_object:
                fcsv.write('{:.6f}, {}, {}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}\n'.format(
                    t,
                    o.id.value,
                    'obj' + str(o.id.value),
                    o.base.position.x,
                    o.base.position.y,
                    o.base.position.z,
                    o.base.velocity.x,
                    o.base.velocity.y,
                    o.base.velocity.z,
                    o.base.orientation.yaw,
                    o.base.orientation.pitch,
                    o.base.orientation.roll,
                    math.sqrt(o.base.velocity.x**2 + o.base.velocity.x**2),
                    0.0,  # wheel rotation not available
                    0.0   # wheel angle not available
                    )
                )

    def close(self):
        self.file.close()

    def read_next_message(self):
        size_bin = self.file.read(4)
        if len(size_bin) < 4:
            return False
        else:
            msg_size = struct.unpack('I', size_bin)[0]
            msg = self.file.read(msg_size)
            self.osi_msg.ParseFromString(msg)
        return True

if __name__ == "__main__":
    # Create the parser
    parser = argparse.ArgumentParser(description='Read .osi file')

    # Add the arguments
    parser.add_argument('filename', help='osi filename')

    # Execute the parse_args() method
    args = parser.parse_args()

    osi = OSIFile(args.filename)
    osi.save_csv()
    osi.close()
