import argparse
import ctypes
import os

VERSION = 2
REPLAY_FILENAME_SIZE = 512
NAME_LEN = 32


class ObjectStateStructDat(ctypes.Structure):
    _fields_ = [

        # ObjectInfoStruct
        ("id", ctypes.c_int),
        ("model_id", ctypes.c_int),
        ("obj_type", ctypes.c_int),
        ("obj_category", ctypes.c_int),
        ("ctrl_type", ctypes.c_int),
        ("time", ctypes.c_float),
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
        ("scaleMode", ctypes.c_int),
        ("visibilityMask", ctypes.c_int),
        ("rgb", ctypes.c_ubyte * 52),

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
        ('version', ctypes.c_int),
        ('odr_filename', ctypes.c_char * REPLAY_FILENAME_SIZE),
        ('model_filename', ctypes.c_char * REPLAY_FILENAME_SIZE),
    ]

class DATFile():
    def __init__(self, filename):
        if not os.path.isfile(filename):
            print('ERROR: dat-file not found: {}'.format(filename))
            return
        try:
            self.file = open(filename, 'rb')
        except OSError:
            print('ERROR: Could not open file {} for reading'.format(filename))
            raise

        self.header = DATHeader.from_buffer_copy(self.file.read(ctypes.sizeof(DATHeader)))
        self.filename = filename
        self.version = self.header.version
        self.odr_filename = self.header.odr_filename.decode('utf-8')
        self.model_filename = self.header.model_filename.decode('utf-8')
        self.labels = [field[0] for field in ObjectStateStructDat._fields_]
        self.data = []

        if (self.version != VERSION):
            print('Version mismatch. {} is version {} while supported version is: {}'.format(
                filename, self.version, VERSION)
            )
            exit(-1)

        # Read and print all rows of data
        while (True):
            buffer = self.file.read(ctypes.sizeof(ObjectStateStructDat))
            if len(buffer) < ctypes.sizeof(ObjectStateStructDat):
                break
            self.data.append(ObjectStateStructDat.from_buffer_copy(buffer))

    def get_header_line(self):
        return 'Version: {}, OpenDRIVE: {}, 3DModel: {}'.format(
                self.version,
                self.odr_filename,
                self.model_filename
            )

    def get_labels_line(self):
        return 'time, id, name, x, y, z, h, p, r, speed, wheel_angle, wheel_rot'

    def get_data_line(self, data):
        return '{:.3f}, {}, {}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}'.format(
                data.time,
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


    def get_labels_line_extended(self):
        return 'time, id, name, x, y, z, h, p, r, roadId, laneId, offset, t, s,\
speed, wheel_angle, wheel_rot, day_light, low_beam, high_beam, fog_light_front, fog_light_rear, brake_light, \
ind_left, ind_right, reversing_light, license_plate, special_pur_light, fog_light, warning_light'

    def get_data_line_extended(self, data):
        str = '{:.3f}, {}, {}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {}, {}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}'.format(
                data.time,
                data.id,
                data.name.decode('utf-8'),
                data.x,
                data.y,
                data.z,
                data.h,
                data.p,
                data.r,
                data.roadId,
                data.laneId,
                data.offset,
                data.t,
                data.s,
                data.speed,
                data.wheel_angle,
                data.wheel_rot
            )

        for i in range(12):
            str += ', #{:02X}{:02X}{:02X}-{:02X}'.format(
                    data.rgb[i*4 + 0], data.rgb[i*4 + 1], data.rgb[i*4 + 2], data.rgb[i*4 + 3]
                )
        return str

    def get_labels_line_array(self):
        return [
            "time",
            "id",
            "name",
            "model_id",
            "obj_type",
            "obj_category",
            "ctrl_type",
            "x",
            "y",
            "z",
            "h",
            "p",
            "r",
            "speed",
            "wheel_angle",
            "wheel_rot",
            "centerOffsetX",
            "centerOffsetY",
            "centerOffsetZ",
            "width",
            "length",
            "height",
            "scaleMode",
            "visibilityMask",
            "roadId",
            "laneId",
            "offset",
            "t",
            "s"];

    def get_data_line_array(self, data):
        return [
            data.time,
            data.id,
            data.name.decode('utf-8'),
            data.model_id,
            data.obj_type,
            data.obj_category,
            data.ctrl_type,
            data.x,
            data.y,
            data.z,
            data.h,
            data.p,
            data.r,
            data.speed,
            data.wheel_angle,
            data.wheel_rot,
            data.centerOffsetX,
            data.centerOffsetY,
            data.centerOffsetZ,
            data.width,
            data.length,
            data.height,
            data.scaleMode,
            data.visibilityMask,
            data.roadId,
            data.laneId,
            data.offset,
            data.t,
            data.s];


    def print_csv(self, extended = False, include_file_refs = True):

        # Print header
        if include_file_refs:
            print(self.get_header_line())

        # Print column headings / value types
        if extended:
            print(self.get_labels_line_extended())
        else:
            print(self.get_labels_line())

        # Read and print all rows of data
        for data in self.data:
            if extended:
                print(self.get_data_line_extended(data))
            else:
                print(self.get_data_line(data))

    def save_csv(self, extended = False, include_file_refs = True):
        csvfile = os.path.splitext(self.filename)[0] + '.csv'
        try:
            fcsv = open(csvfile, 'w')
        except OSError:
            print('ERROR: Could not open file {} for writing'.format(csvfile))
            raise

        # Save column headings / value types
        if include_file_refs:
            fcsv.write(self.get_header_line() + '\n')

        # Save column headings / value types
        if extended:
            fcsv.write(self.get_labels_line_extended() + '\n')
        else:
            fcsv.write(self.get_labels_line() + '\n')

        # Read and save all rows of data
        for data in self.data:
            if extended:
                fcsv.write(self.get_data_line_extended(data) + '\n')
            else:
                fcsv.write(self.get_data_line(data) + '\n')

        fcsv.close()

    def save_dat(self, filename):
        try:
            fdat = open(filename, 'wb')
        except OSError:
            print('ERROR: Could not open file {} for writing'.format(filename))
            raise

        fdat.write(self.header)

        for d in self.data:
            fdat.write(d)

        fdat.close()

    def close(self):
        self.file.close()

if __name__ == "__main__":
    # Create the parser
    parser = argparse.ArgumentParser(description='Read and print .dat file')

    # Add the arguments
    parser.add_argument('filename', help='dat filename')
    parser.add_argument('--extended', '-e', action='store_true', help='add road coordinates')
    parser.add_argument('--file_refs', '-r', action='store_true', help='include odr and model file references')

    # Execute the parse_args() method
    args = parser.parse_args()

    dat = DATFile(args.filename)
    dat.print_csv(args.extended, args.file_refs)
    dat.close()
