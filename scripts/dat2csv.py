import numpy as np
import argparse
import ctypes

REPLAY_FILENAME_SIZE = 128
NAME_LEN = 32

class ObjectStateStructDat(ctypes.Structure):
    _fields_ = [
        
        # ObjectInfoStruct
        ("id", ctypes.c_int),
        ("model_id", ctypes.c_int),
        ("obj_type", ctypes.c_int),
        ("obj_category", ctypes.c_int),
        ("ctrl_type", ctypes.c_int),
        ("timestamp", ctypes.c_float),
        ('name', ctypes.c_char * NAME_LEN),        
        ("speed", ctypes.c_float),
        ("wheel_angle", ctypes.c_float),
        ("wheel_rot", ctypes.c_float),
        ("centerOffsetX", ctypes.c_float),
        ("centerOffsetY", ctypes.c_float),
        ("centerOffsetZ", ctypes.c_float),
        ("width", ctypes.c_float),
        ("length", ctypes.c_float),
        ("height", ctypes.c_float),
        
        # ObjectPositionStruct
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float),
        ("h", ctypes.c_float),
        ("p", ctypes.c_float),
        ("r", ctypes.c_float),
        ("roadId", ctypes.c_int),
        ("laneId", ctypes.c_int),
        ("offset", ctypes.c_float),
        ("t", ctypes.c_float),
        ("s", ctypes.c_float),
    ]

class DATHeader(ctypes.Structure):
    _fields_ = [
        ('odr_filename', ctypes.c_char * REPLAY_FILENAME_SIZE),
        ('model_filename', ctypes.c_char * REPLAY_FILENAME_SIZE),
    ]


# Create the parser
parser = argparse.ArgumentParser(description='Convert esmini .dat file to .csv')

# Add the arguments
parser.add_argument('filename', help='dat filename')

# Execute the parse_args() method
args = parser.parse_args()

file = open(args.filename, 'rb')
buffer = file.read(2 * REPLAY_FILENAME_SIZE)

h = DATHeader.from_buffer_copy(buffer)
print('OpenDRIVE: {}, 3DModel: {}'.format(
    h.odr_filename.decode('utf-8'), h.model_filename.decode('utf-8'))
)

# Print column headings / value types
print('time, id, name, x, y, z, h, p, r, speed, wheel_angle, wheel_rot')

# Read and print all rows of data
while (True):

    buffer = file.read(ctypes.sizeof(ObjectStateStructDat))
    if len(buffer) < ctypes.sizeof(ObjectStateStructDat):
        break

    data = ObjectStateStructDat.from_buffer_copy(buffer)

    print('{:.3f}, {}, {}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}'.format(
        data.timestamp, 
        data.id,
		data.name.decode('utf-8'),
		data.x,
		data.y,
		data.z,
		data.h,
		data.p,
		data.r,
		data.speed,
		data.wheel_angle,
		data.wheel_rot)
    )

file.close()