import argparse
import struct
import os
import enum

VERSION_MAJOR = 1
VERSION_MINOR = 0

class PacketId(enum.Enum):
    """Enum for packet IDs."""
    OBJ_ID          = 0
    MODEL_ID        = 1
    OBJ_TYPE        = 2
    OBJ_CATEGORY    = 3
    CTRL_TYPE       = 4
    TIMESTAMP       = 5
    NAME            = 6
    SPEED           = 7
    WHEEL_ANGLE     = 8
    WHEEL_ROT       = 9
    BOUNDING_BOX    = 10
    SCALE_MODE      = 11
    VISIBILITY_MASK = 12
    POSE            = 13
    ROAD_ID         = 14
    LANE_ID         = 15
    POS_OFFSET      = 16
    POS_T           = 17
    POS_S           = 18
    OBJ_DELETED     = 19
    OBJ_ADDED       = 20
    END_OF_SCENARIO = 21

class DataType:
    """Enum for data types used in the .dat file."""
    double = '<d'
    int32 = '<i'
    uint32 = '<I'
    int16 = '<h'
    uint16 = '<H'
    float = '<f'
    char = '<c'

def read_dtype(file, data_type):
    """Read data from the file based on the specified data type."""
    if data_type == DataType.double:
        return struct.unpack(DataType.double, file.read(8))[0]
    elif data_type == DataType.int32:
        return struct.unpack(DataType.int32, file.read(4))[0]
    elif data_type == DataType.uint32:
        return struct.unpack(DataType.uint32, file.read(4))[0]
    elif data_type == DataType.int16:
        return struct.unpack(DataType.int16, file.read(2))[0]
    elif data_type == DataType.uint16:
        return struct.unpack(DataType.uint16, file.read(2))[0]
    elif data_type == DataType.float:
        return struct.unpack(DataType.float, file.read(4))[0]
    elif data_type == DataType.char:
        return struct.unpack(DataType.char, file.read(1))[0]
    else:
        raise ValueError(f"Unsupported data type: {data_type}")

def read_packet_header(file) -> tuple:
    """ Read the header of a packet """
    header_format = '<II'
    header_size = struct.calcsize(header_format)
    header_data = file.read(header_size)
    if (len(header_data) != header_size):
        raise EOFError("Unexpected end of file while reading packet header")
    packet_id, _ = struct.unpack(header_format, header_data)
    return packet_id

def read_string_packet(file):
    """Read a string packet from the file."""
    size = read_dtype(file, DataType.uint32)
    string_bytes = file.read(size)
    return string_bytes.decode('utf-8')

def read_dat_header(file):
    """Read the header of a .dat file."""
    version_major = read_dtype(file, DataType.uint32)
    version_minor = read_dtype(file, DataType.uint32)
    odr_filename = read_string_packet(file)
    model_filename = read_string_packet(file)
    return (version_major, version_minor, odr_filename, model_filename)

ObjectStateStructDat = {
    "id": {},
    "model_id": {},
    "obj_type": {},
    "obj_category": {},
    "ctrl_type": {},
    "time": {},
    "name": "",
    "speed": {},
    "wheel_angle": {},
    "wheel_rot": {},
    "centerOffsetX": {},
    "centerOffsetY": {},
    "centerOffsetZ": {},
    "width": {},
    "length": {},
    "height": {},
    "scaleMode": {},
    "visibilityMask": {},
    "active": {},
    "x": {},
    "y": {},
    "z": {},
    "h": {},
    "p": {},
    "r": {},
    "roadId": {},
    "laneId": {},
    "offset": {},
    "t": {},
    "s": {}
}

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

        self.version_major, self.version_minor, self.odr_filename, self.model_filename = read_dat_header(self.file)
        print(f"Reading {filename} with version {self.get_header_line()}")

        if (self.version_major != VERSION_MAJOR or self.version_minor != VERSION_MINOR):
            print('Version mismatch. {} is version {}.{} while supported version is: {}.{}'.format(
                filename, self.version_major, self.version_minor, VERSION_MAJOR, VERSION_MINOR)
            )
            exit(-1)

        self.filename = filename
        self.file_size = os.path.getsize(filename)

        self.labels = [key for key in ObjectStateStructDat.items()]
        self.data = []

        while self.file.tell() < self.file_size:
            p_id = read_packet_header(self.file)
            if p_id == PacketId.TIMESTAMP.value:
                print(f"Timestamp: {read_dtype(self.file, DataType.double)}")
            elif p_id == PacketId.OBJ_ID.value:
                obj_id = read_dtype(self.file, DataType.int32)
                print(f"Object ID: {obj_id}")
            elif p_id == PacketId.MODEL_ID.value:
                model_id = read_dtype(self.file, DataType.int32)
                print(f"Model ID: {model_id}")
            elif p_id == PacketId.OBJ_TYPE.value:
                obj_type = read_dtype(self.file, DataType.int32)
                print(f"Object Type: {obj_type}")
            elif p_id == PacketId.OBJ_CATEGORY.value:
                obj_category = read_dtype(self.file, DataType.int32)
                print(f"Object Category: {obj_category}")
            elif p_id == PacketId.CTRL_TYPE.value:
                ctrl_type = read_dtype(self.file, DataType.int32)
                print(f"Control Type: {ctrl_type}")
            elif p_id == PacketId.NAME.value:
                name = read_string_packet(self.file)
                print(f"Name: {name}")
            elif p_id == PacketId.SPEED.value:
                speed = read_dtype(self.file, DataType.double)
                print(f"Speed: {speed}")
            elif p_id == PacketId.WHEEL_ANGLE.value:
                wheel_angle = read_dtype(self.file, DataType.double)
                print(f"Wheel Angle: {wheel_angle}")
            elif p_id == PacketId.WHEEL_ROT.value:
                wheel_rot = read_dtype(self.file, DataType.double)
                print(f"Wheel Rotation: {wheel_rot}")
            elif p_id == PacketId.POSE.value:
                x = read_dtype(self.file, DataType.double)
                y = read_dtype(self.file, DataType.double)
                z = read_dtype(self.file, DataType.double)
                h = read_dtype(self.file, DataType.double)
                p = read_dtype(self.file, DataType.double)
                r = read_dtype(self.file, DataType.double)
                print(f"Pose: x={x}, y={y}, z={z}, h={h}, p={p}, r={r}")
            elif p_id == PacketId.BOUNDING_BOX.value:
                centerOffsetX = read_dtype(self.file, DataType.double)
                centerOffsetY = read_dtype(self.file, DataType.double)
                centerOffsetZ = read_dtype(self.file, DataType.double)
                width = read_dtype(self.file, DataType.double)
                length = read_dtype(self.file, DataType.double)
                height = read_dtype(self.file, DataType.double)
                print(f"Bounding Box: centerOffsetX={centerOffsetX}, centerOffsetY={centerOffsetY}, centerOffsetZ={centerOffsetZ}, width={width}, length={length}, height={height}")
            elif p_id == PacketId.SCALE_MODE.value:
                scale_mode = read_dtype(self.file, DataType.int32)
                print(f"Scale Mode: {scale_mode}")
            elif p_id == PacketId.VISIBILITY_MASK.value:
                visibility_mask = read_dtype(self.file, DataType.int32)
                print(f"Visibility Mask: {visibility_mask}")
            elif p_id == PacketId.ROAD_ID.value:
                road_id = read_dtype(self.file, DataType.uint32)
                print(f"Road ID: {road_id}")
            elif p_id == PacketId.LANE_ID.value:
                lane_id = read_dtype(self.file, DataType.int32)
                print(f"Lane ID: {lane_id}")
            elif p_id == PacketId.POS_OFFSET.value:
                offset = read_dtype(self.file, DataType.double)
                print(f"Position Offset: {offset}")
            elif p_id == PacketId.POS_T.value:
                t = read_dtype(self.file, DataType.double)
                print(f"Position T: {t}")
            elif p_id == PacketId.POS_S.value:
                s = read_dtype(self.file, DataType.double)
                print(f"Position S: {s}")
            elif p_id == PacketId.OBJ_DELETED.value:
                print(f"Object Deleted:")

    def get_header_line(self):
        return f'Version: {self.version_major}.{self.version_minor}, OpenDRIVE: {self.odr_filename}, 3DModel: {self.model_filename}'

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
                data.wheel_rot
            )

    def get_labels_line_extended(self):
        return 'time, id, name, x, y, z, h, p, r, roadId, laneId, offset, t, s, speed, wheel_angle, wheel_rot'

    def get_data_line_extended(self, data):
        return '{:.3f}, {}, {}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {}, {}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {}, {}'.format(
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
                data.wheel_rot,
                data.centerOffsetX,
                data.centerOffsetY,
                data.centerOffsetZ,
                data.width,
                data.length,
                data.height,
                data.scaleMode,
                data.visibilityMask
            )

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
