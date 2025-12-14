import os
import struct
import argparse
import math

from osi3.osi_groundtruth_pb2 import *

class OSIFile():
    def __init__(self, filename):
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
        fcsv.write('time, id, name, type, x, y, z, vx, vy, vz, ax, ay, az, h, p, r, vh, vp, vr, ah, ap, ar, speed, wheel_angle, wheel_rot\n')

        # write data
        while self.read_next_message():
            t = self.osi_msg.timestamp.seconds + self.osi_msg.timestamp.nanos * 1e-9
            for o in self.osi_msg.moving_object:
                if o.type == 0:
                    type = 'UNKNOWN'
                elif o.type == 1:
                    type = 'OTHER'
                elif o.type == 2:
                    type = str(o.vehicle_classification)[11:-1]
                elif o.type == 3:
                    type = 'PEDESTRIAN'
                elif o.type == 4:
                    type = 'ANIMAL'
                else:
                    type = 'ERROR'
                fcsv.write('{:.6f}, {}, {}, {}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}\n'.format(
                    t,
                    o.id.value,
                    'obj' + str(o.id.value),
                    type,
                    o.base.position.x,
                    o.base.position.y,
                    o.base.position.z,
                    o.base.velocity.x,
                    o.base.velocity.y,
                    o.base.velocity.z,
                    o.base.acceleration.x,
                    o.base.acceleration.y,
                    o.base.acceleration.z,
                    o.base.orientation.yaw,
                    o.base.orientation.pitch,
                    o.base.orientation.roll,
                    o.base.orientation_rate.yaw,
                    o.base.orientation_rate.pitch,
                    o.base.orientation_rate.roll,
                    o.base.orientation_acceleration.yaw,
                    o.base.orientation_acceleration.pitch,
                    o.base.orientation_acceleration.roll,
                    math.sqrt(o.base.velocity.x**2 + o.base.velocity.y**2),
                    getattr(o.vehicle_attributes, 'wheel_data')[0].orientation.yaw if getattr(o.vehicle_attributes, 'wheel_data') else 0.0,  # only one wheel for now
                    0.0  # wheel rotation not available
                    )
                )

            for o in self.osi_msg.stationary_object:
                fcsv.write('{:.6f}, {}, {}, {}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}, {:.6f}\n'.format(
                    t,
                    o.id.value,
                    'misc_obj' + str(o.id.value),
                    o.classification.type,
                    o.base.position.x,
                    o.base.position.y,
                    o.base.position.z,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    o.base.orientation.yaw,
                    o.base.orientation.pitch,
                    o.base.orientation.roll,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,  # no wheels
                    0.0  # wheel rotation not available
                    )
                )

        fcsv.close()

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
