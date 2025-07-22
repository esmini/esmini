import argparse
import struct
import os
import enum
from collections import defaultdict

VERSION_MAJOR = 1
VERSION_MINOR = 0
        
EPSILON = 0.0001

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


class DATFile():
    ObjectStateStructDat = {
        "id": None,
        "model_id": None,
        "obj_type": None,
        "obj_category": None,
        "ctrl_type": None,
        "time": None,
        "name": "",
        "speed": None,
        "wheel_angle": None,
        "wheel_rot": None,
        "centerOffsetX": None,
        "centerOffsetY": None,
        "centerOffsetZ": None,
        "width": None,
        "length": None,
        "height": None,
        "scaleMode": None,
        "visibilityMask": None,
        "active": None,
        "x": None,
        "y": None,
        "z": None,
        "h": None,
        "p": None,
        "r": None,
        "roadId": None,
        "laneId": None,
        "offset": None,
        "t": None,
        "s": None
    }
    def __init__(self, filename, extended=False):
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

        self.extended = extended

        self.filename = filename
        self.file_size = os.path.getsize(filename)

        self.labels = self.ObjectStateStructDat.keys()
        self.object_ids = set()
        self.object_events_map = defaultdict(list)
        self.object_state_cache = defaultdict()
        self.min_timestep = float('inf')
        self.timestamp = 0.0
        self.end_time = 0.0
        self.id_to_search_idx = {}

        self.data = []

        self.parse_data()

        self.build_data()

    def build_data(self):
        self.object_state_cache.clear()


        dt = self.min_timestep
        # TODO: Allow custom argument for dt
        start_time = 0.0
        for _, events in self.object_events_map.items():
            start_time = min(start_time, events[0]["time"])
        
        for obj_id in self.object_ids:
            self.id_to_search_idx[obj_id] = 0
        
        template = self.ObjectStateStructDat
        if self.extended:
            labels = self.get_labels_line_extended()
        else:
            labels = self.get_labels_line()
        
        t = start_time
        while t <= self.end_time:
            for obj_id in self.object_ids:
                events = self.object_events_map.get(obj_id, [])
                last_state = self.object_state_cache.get(obj_id, template.copy())

                if events:
                    start_idx = self.id_to_search_idx.get(obj_id, 0)
                    for i in range(start_idx, len(events)):
                        if events[i]["time"] <= t + EPSILON:
                            last_state = events[i]
                            self.id_to_search_idx[obj_id] = i
                        else:
                            break

                state = last_state.copy()
                state["time"] = t
                if state["active"]:
                    self.data.append([state[label] for label in labels])
            t += dt

    def parse_data(self):
        while self.file.tell() < self.file_size:
            p_id = read_packet_header(self.file)
            if p_id == PacketId.TIMESTAMP.value:
                self.timestamp = read_dtype(self.file, DataType.double)

            elif p_id == PacketId.OBJ_ID.value:
                if self.ObjectStateStructDat["id"] in self.object_ids:
                    self.object_events_map[self.ObjectStateStructDat["id"]].append(self.ObjectStateStructDat.copy())
                    self.object_state_cache[self.ObjectStateStructDat["id"]] = self.ObjectStateStructDat.copy()

                obj_id = read_dtype(self.file, DataType.int32)
                self.ObjectStateStructDat["id"] = obj_id

                if obj_id in self.object_state_cache:
                    self.ObjectStateStructDat = self.object_state_cache[obj_id]
                    
                    if self.min_timestep == float('inf'):
                        self.min_timestep = self.timestamp - self.ObjectStateStructDat["time"]
                    else:
                        dt = self.timestamp - self.ObjectStateStructDat["time"]
                        if not (abs(dt) < 0.001):
                            self.min_timestep = min(self.min_timestep, dt)
                
                self.ObjectStateStructDat["time"] = self.timestamp
                self.ObjectStateStructDat["active"] = True
                self.ObjectStateStructDat["id"] = obj_id
                self.object_ids.add(obj_id)
                
            elif p_id == PacketId.MODEL_ID.value:
                model_id = read_dtype(self.file, DataType.int32)
                self.ObjectStateStructDat["model_id"] = model_id
            elif p_id == PacketId.OBJ_TYPE.value:
                obj_type = read_dtype(self.file, DataType.int32)
                self.ObjectStateStructDat["obj_type"] = obj_type
            elif p_id == PacketId.OBJ_CATEGORY.value:
                obj_category = read_dtype(self.file, DataType.int32)
                self.ObjectStateStructDat["obj_category"] = obj_category
            elif p_id == PacketId.CTRL_TYPE.value:
                ctrl_type = read_dtype(self.file, DataType.int32)
                self.ObjectStateStructDat["ctrl_type"] = ctrl_type
            elif p_id == PacketId.NAME.value:
                name = read_string_packet(self.file)
                self.ObjectStateStructDat["name"] = name
            elif p_id == PacketId.SPEED.value:
                speed = read_dtype(self.file, DataType.double)
                self.ObjectStateStructDat["speed"] = speed
            elif p_id == PacketId.WHEEL_ANGLE.value:
                wheel_angle = read_dtype(self.file, DataType.double)
                self.ObjectStateStructDat["wheel_angle"] = wheel_angle
            elif p_id == PacketId.WHEEL_ROT.value:
                wheel_rot = read_dtype(self.file, DataType.double)
                self.ObjectStateStructDat["wheel_rot"] = wheel_rot
            elif p_id == PacketId.POSE.value:
                self.ObjectStateStructDat["x"] = read_dtype(self.file, DataType.double)
                self.ObjectStateStructDat["y"] = read_dtype(self.file, DataType.double)
                self.ObjectStateStructDat["z"] = read_dtype(self.file, DataType.double)
                self.ObjectStateStructDat["h"] = read_dtype(self.file, DataType.double)
                self.ObjectStateStructDat["p"] = read_dtype(self.file, DataType.double)
                self.ObjectStateStructDat["r"] = read_dtype(self.file, DataType.double)
            elif p_id == PacketId.BOUNDING_BOX.value:
                self.ObjectStateStructDat["centerOffsetX"] = read_dtype(self.file, DataType.double)
                self.ObjectStateStructDat["centerOffsetY"] = read_dtype(self.file, DataType.double)
                self.ObjectStateStructDat["centerOffsetZ"] = read_dtype(self.file, DataType.double)
                self.ObjectStateStructDat["width"] = read_dtype(self.file, DataType.double)
                self.ObjectStateStructDat["length"] = read_dtype(self.file, DataType.double)
                self.ObjectStateStructDat["height"] = read_dtype(self.file, DataType.double)
            elif p_id == PacketId.SCALE_MODE.value:
                self.ObjectStateStructDat["scaleMode"] = read_dtype(self.file, DataType.int32)
            elif p_id == PacketId.VISIBILITY_MASK.value:
                self.ObjectStateStructDat["visibilityMask"] = read_dtype(self.file, DataType.int32)
            elif p_id == PacketId.ROAD_ID.value:
                self.ObjectStateStructDat["roadId"] = read_dtype(self.file, DataType.uint32)
            elif p_id == PacketId.LANE_ID.value:
                self.ObjectStateStructDat["laneId"] = read_dtype(self.file, DataType.int32)
            elif p_id == PacketId.POS_OFFSET.value:
                self.ObjectStateStructDat["offset"] = read_dtype(self.file, DataType.double)
            elif p_id == PacketId.POS_T.value:
                self.ObjectStateStructDat["t"] = read_dtype(self.file, DataType.double)
            elif p_id == PacketId.POS_S.value:
                self.ObjectStateStructDat["s"] = read_dtype(self.file, DataType.double)
            elif p_id == PacketId.OBJ_DELETED.value:
                self.ObjectStateStructDat["active"] = False
            elif p_id == PacketId.END_OF_SCENARIO.value:
                self.object_events_map[self.ObjectStateStructDat["id"]].append(self.ObjectStateStructDat.copy())
                self.end_time = read_dtype(self.file, DataType.double)

    def get_header_line(self):
        return f'Version: {self.version_major}.{self.version_minor}, OpenDRIVE: {self.odr_filename}, 3DModel: {self.model_filename}'

    def get_labels_line(self):
        return ['time', 'id', 'name', 'x', 'y', 'z', 'h', 'p', 'r', 'speed', 'wheel_angle', 'wheel_rot']

    def get_data_line(self, data):
        """ Will contain extended data if self.extended is True """
        return ', '.join(f"{x:.3f}" if isinstance(x, float) else str(x) for x in data)

    def get_labels_line_extended(self):
        return ['time', 'id', 'name', 'x', 'y', 'z', 'h', 'p', 'r', 'roadId', 'laneId', 'offset', 't', 's', 'speed', 'wheel_angle', 'wheel_rot']

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
            data["time"],
            data["id"],
            data["name"].decode('utf-8'),
            data["modelId"],
            data["objType"],
            data["objCategory"],
            data["ctrlType"],
            data["x"],
            data["y"],
            data["z"],
            data["h"],
            data["p"],
            data["r"],
            data["speed"],
            data["wheel_angle"],
            data["wheel_rot"],
            data["centerOffsetX"],
            data["centerOffsetY"],
            data["centerOffsetZ"],
            data["width"],
            data["length"],
            data["height"],
            data["scaleMode"],
            data["visibilityMask"],
            data["roadId"],
            data["laneId"],
            data["offset"],
            data["t"],
            data["s"]];

    # TODO
    # def print_csv(self, extended = False, include_file_refs = True):

    #     # Print header
    #     if include_file_refs:
    #         print(self.get_header_line())

    #     # Print column headings / value types
    #     if extended:
    #         print(self.get_labels_line_extended())
    #     else:
    #         print(self.get_labels_line())

    #     # Read and print all rows of data
    #     for data in self.data:
    #         if extended:
    #             print(self.get_data_line_extended(data))
    #         else:
    #             print(self.get_data_line(data))

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
            fcsv.write(', '.join(self.get_labels_line_extended()) + '\n')
        else:
            fcsv.write(', '.join(self.get_labels_line()) + '\n')

        # Read and save all rows of data
        for data in self.data:
            fcsv.write(self.get_data_line(data) + '\n')

        fcsv.close()

    # TODO
    # def save_dat(self, filename):
    #     try:
    #         fdat = open(filename, 'wb')
    #     except OSError:
    #         print('ERROR: Could not open file {} for writing'.format(filename))
    #         raise

    #     fdat.write(self.header)

    #     for d in self.data:
    #         fdat.write(d)

    #     fdat.close()

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

    dat = DATFile(args.filename, args.extended)
    dat.print_csv(args.file_refs)
    dat.close()
