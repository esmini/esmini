import argparse
import struct
import os
import enum
from collections import defaultdict
import copy
import bisect

VERSION_MAJOR = 1
VERSION_MINOR = 0
        
SMALL_NUMBER = 1e-6
LARGE_NUMBER = 1e10

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
    DT              = 21
    END_OF_SCENARIO = 22
    PACKET_ID_SIZE  = 23

class Pose:
    def __init__(self):
        self.x = None
        self.y = None
        self.z = None
        self.h = None
        self.p = None
        self.r = None

class BoundingBox:
    def __init__(self):
        self.center_offset_x = None
        self.center_offset_y = None
        self.center_offset_z = None
        self.width = None
        self.length = None
        self.height = None

class DataType:
    """Class for data types used in the .dat file."""
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

def is_near(x, y):
    return abs(x - y) < SMALL_NUMBER

class Timeline():
    """
    Contains all the timeline data for a specific variable
    """
    def __init__(self):
        self.values = []
        self.last_index = 0
        self.last_time = LARGE_NUMBER

    def get_value_incremental(self, time):
        if (len(self.values) == 0):
            print("ERROR, no values available")
            exit(-1)

        idx = self.last_index
        desired_dt = time - self.values[self.last_index][0]

        if is_near(self.last_time, time):
            return self.values[self.last_index][1]
        
        if time >= self.values[idx][0]: # We search forward in time
            while idx + 1 < len(self.values):
                step = self.values[idx + 1][0] - self.values[self.last_index][0]
                if desired_dt + SMALL_NUMBER < step - SMALL_NUMBER:
                    break
                idx += 1
        
        self.last_index = idx
        self.last_time = time

        return self.values[idx][1]
    
    def get_index_binary(self, time):
        if not self.values:
            raise ValueError(f"Timeline is empty, cannot get value at time {time}")

        if self.last_time is not None and is_near(self.last_time, time):
            return self.last_index

        times = [t for t, _ in self.values]

        # Choose search range depending on direction
        if time >= self.values[self.last_index][0]:
            # searching forward
            search_start = self.last_index
        else:
            # searching backward
            search_start = 0

        # upper_bound equivalent in Python
        # bisect_right returns the first index > time
        idx = bisect.bisect_right(times, time, lo=search_start)

        return idx
    
    def get_value_binary(self, time, upper):
        if len(self.values) == 0:
            print("ERROR, no values available")
            exit(-1)

        if upper:
            # Get the upper bound
            idx = bisect.bisect_right(self.values, (time, float('inf')))
        else:
            # Get the lower bound
            idx = bisect.bisect_left(self.values, (time, float('-inf')))

        if idx == 0:
            return self.values[0][1]
        elif idx == len(self.values):
            return self.values[-1][1]
        else:
            return self.values[idx][1]



class PropertyTimeline():
    """
    Contains all the timelines
    """
    def __init__(self):
        self.model_id = Timeline()
        self.obj_type = Timeline()
        self.obj_category = Timeline()
        self.ctrl_type = Timeline()
        self.name = Timeline()
        self.speed = Timeline()
        self.wheel_angle = Timeline()
        self.wheel_rot = Timeline()
        self.bounding_box = Timeline()
        self.scale_mode = Timeline()
        self.visibility_mask = Timeline()
        self.pose = Timeline()
        self.road_id = Timeline()
        self.lane_id = Timeline()
        self.pos_offset = Timeline()
        self.pos_t = Timeline()
        self.pos_s = Timeline()
        self.active = Timeline()

        self.last_restart_time = -1.0

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

        if (self.version_major != VERSION_MAJOR or self.version_minor != VERSION_MINOR):
            print('Version mismatch. {} is version {}.{} while supported version is: {}.{}'.format(
                filename, self.version_major, self.version_minor, VERSION_MAJOR, VERSION_MINOR)
            )
            exit(-1)

        self.extended = extended

        self.filename = filename
        self.file_size = os.path.getsize(filename)

        self.labels = self.ObjectStateStructDat.keys()
        self.current_object_id = None
        self.current_object_timeline = None
        self.objects_timeline = defaultdict()
        self.object_state_cache = defaultdict()
        self.dt = Timeline()
        self.current_timestamp = 0.0
        self.ghost_ghost_counter = -1
        self.timestamps = []
        self.end_time = 0.0

        self.data = []

        self.parse_data()
        self.fill_timestamps()
        self.build_csv()

    def parse_data(self):
        """Parse the .dat file and extract object states.
           Packets are read in the order they are likely to appear in the file.
        """
        while self.file.tell() < self.file_size:
            p_id = read_packet_header(self.file)

            # TIMESTAMP packet
            if p_id == PacketId.TIMESTAMP.value:
                self.current_timestamp = read_dtype(self.file, DataType.double)

                if len(self.timestamps) == 0 or self.current_timestamp < SMALL_NUMBER or self.current_timestamp > self.timestamps[-1]:
                    self.timestamps.append(self.current_timestamp)

            # OBJ_ID packet
            elif p_id == PacketId.OBJ_ID.value:
                self.current_object_id = read_dtype(self.file, DataType.int32)

                if self.objects_timeline.get(self.current_object_id) is None:
                    self.objects_timeline[self.current_object_id] = PropertyTimeline()
                    self.current_object_timeline = self.objects_timeline[self.current_object_id]
                    if self.current_timestamp > 0.0:
                        self.current_object_timeline.active.values.append([0.0, False])
                        self.current_object_timeline.active.values.append([self.current_timestamp, True])
                    else:
                        self.current_object_timeline.active.values.append([self.current_timestamp, True])
                else:
                    self.current_object_timeline = self.objects_timeline[self.current_object_id]
                    if self.current_object_timeline.active.values[-1][1] == False:
                        self.current_object_timeline.active.values.append([self.current_timestamp, True])

            # POSE packet
            elif p_id == PacketId.POSE.value:
                pose = Pose()
                for k in ["x", "y", "z", "h", "p", "r"]:
                    setattr(pose, k, read_dtype(self.file, DataType.float))
                self.add_to_timeline(self.current_object_timeline.pose, pose)

            elif p_id == PacketId.DT.value:
                self.dt.values.append([self.current_timestamp, read_dtype(self.file, DataType.double)])
            elif p_id == PacketId.SPEED.value:
                self.add_to_timeline(self.current_object_timeline.speed, read_dtype(self.file, DataType.float))
            elif p_id == PacketId.WHEEL_ANGLE.value:
                self.add_to_timeline(self.current_object_timeline.wheel_angle, read_dtype(self.file, DataType.float))
            elif p_id == PacketId.WHEEL_ROT.value:
                self.add_to_timeline(self.current_object_timeline.wheel_rot, read_dtype(self.file, DataType.float))
            elif p_id == PacketId.POS_OFFSET.value:
                self.add_to_timeline(self.current_object_timeline.pos_offset, read_dtype(self.file, DataType.float))
            elif p_id == PacketId.POS_T.value:
                self.add_to_timeline(self.current_object_timeline.pos_t, read_dtype(self.file, DataType.float))
            elif p_id == PacketId.POS_S.value:
                self.add_to_timeline(self.current_object_timeline.pos_s, read_dtype(self.file, DataType.float))
            elif p_id == PacketId.MODEL_ID.value:
                self.current_object_timeline.model_id.values.append([self.current_timestamp, read_dtype(self.file, DataType.int32)])
            elif p_id == PacketId.OBJ_TYPE.value:
                self.current_object_timeline.obj_type.values.append([self.current_timestamp, read_dtype(self.file, DataType.int32)])
            elif p_id == PacketId.OBJ_CATEGORY.value:
                self.current_object_timeline.obj_category.values.append([self.current_timestamp, read_dtype(self.file, DataType.int32)])
            elif p_id == PacketId.CTRL_TYPE.value:
                self.current_object_timeline.ctrl_type.values.append([self.current_timestamp, read_dtype(self.file, DataType.int32)])
            elif p_id == PacketId.SCALE_MODE.value:
                self.current_object_timeline.scale_mode.values.append([self.current_timestamp, read_dtype(self.file, DataType.int32)])
            elif p_id == PacketId.VISIBILITY_MASK.value:
                self.add_to_timeline(self.current_object_timeline.visibility_mask, read_dtype(self.file, DataType.int32))
            elif p_id == PacketId.ROAD_ID.value:
                self.add_to_timeline(self.current_object_timeline.road_id, read_dtype(self.file, DataType.uint32))
            elif p_id == PacketId.LANE_ID.value:
                self.add_to_timeline(self.current_object_timeline.lane_id, read_dtype(self.file, DataType.int32))
            elif p_id == PacketId.NAME.value:
                name = read_string_packet(self.file)
                self.current_object_timeline.name.values.append([self.current_timestamp, name])
            elif p_id == PacketId.BOUNDING_BOX.value:
                bb = BoundingBox()
                for k in ["center_offset_x", "center_offset_y", "center_offset_z", "width", "length", "height"]:
                    setattr(bb, k, read_dtype(self.file, DataType.float))
                self.current_object_timeline.bounding_box.values.append([self.current_timestamp, bb])

            # OBJ_DELETED packet
            elif p_id == PacketId.OBJ_DELETED.value:
                self.current_object_timeline.active.values.append([self.current_timestamp, False])
            
            # END_OF_SCENARIO packet
            elif p_id == PacketId.END_OF_SCENARIO.value:
                self.end_time = read_dtype(self.file, DataType.double)

    def add_to_timeline(self, timeline: PropertyTimeline, data):
        if len(self.current_object_timeline.ctrl_type.values) != 0 and self.current_object_timeline.ctrl_type.values[-1][1] != 100:
            timeline.values.append([self.current_timestamp, data])
            return
        
        if len(timeline.values) != 0 and self.current_timestamp < timeline.values[-1][0]:
            obj_tl = self.objects_timeline.get(self.current_object_id)

            if not is_near(obj_tl.last_restart_time, self.current_timestamp):
                ghost_ghost_id = self.current_object_id * self.ghost_ghost_counter
                
                if self.objects_timeline.get(ghost_ghost_id) is None:
                    self.objects_timeline[ghost_ghost_id] = copy.deepcopy(obj_tl)
                    ghost_tl = self.objects_timeline[ghost_ghost_id]
                    ghost_tl.active.values[0][1] = False
                    ghost_tl.name.values[0][1] += f"_{self.ghost_ghost_counter}"
                    ghost_tl.active.values.append([self.current_timestamp, True])
                    ghost_tl.active.values.append([self.timestamps[-1], False])

                    self.ghost_ghost_counter -= 1
                    
                obj_tl.last_restart_time = self.current_timestamp        

            slice_idx = timeline.get_index_binary(self.current_timestamp)
            timeline.values = timeline.values[0:slice_idx]
        
        timeline.values.append([self.current_timestamp, data])

    def get_object_state_struct_at_time(self, obj_id, t):
        timeline = self.objects_timeline.get(obj_id)

        self.ObjectStateStructDat["id"] = obj_id
        self.ObjectStateStructDat["time"] = t
        self.ObjectStateStructDat["model_id"] = timeline.model_id.get_value_incremental(t)
        self.ObjectStateStructDat["obj_type"] = timeline.obj_type.get_value_incremental(t)
        self.ObjectStateStructDat["obj_category"] = timeline.obj_category.get_value_incremental(t)
        self.ObjectStateStructDat["ctrl_type"] = timeline.ctrl_type.get_value_incremental(t)
        self.ObjectStateStructDat["name"] = timeline.name.get_value_incremental(t)
        self.ObjectStateStructDat["speed"] = timeline.speed.get_value_incremental(t)
        self.ObjectStateStructDat["wheel_angle"] = timeline.wheel_angle.get_value_incremental(t)
        self.ObjectStateStructDat["wheel_rot"] = timeline.wheel_rot.get_value_incremental(t)
        self.ObjectStateStructDat["centerOffsetX"] = timeline.bounding_box.get_value_incremental(t).center_offset_x
        self.ObjectStateStructDat["centerOffsetY"] = timeline.bounding_box.get_value_incremental(t).center_offset_y
        self.ObjectStateStructDat["centerOffsetZ"] = timeline.bounding_box.get_value_incremental(t).center_offset_z
        self.ObjectStateStructDat["width"] = timeline.bounding_box.get_value_incremental(t).width
        self.ObjectStateStructDat["length"] = timeline.bounding_box.get_value_incremental(t).length
        self.ObjectStateStructDat["height"] = timeline.bounding_box.get_value_incremental(t).height
        self.ObjectStateStructDat["scaleMode"] = timeline.scale_mode.get_value_incremental(t)
        self.ObjectStateStructDat["visibilityMask"] = timeline.visibility_mask.get_value_incremental(t)
        self.ObjectStateStructDat["active"] = timeline.active.get_value_incremental(t)
        self.ObjectStateStructDat["x"] = timeline.pose.get_value_incremental(t).x
        self.ObjectStateStructDat["y"] = timeline.pose.get_value_incremental(t).y
        self.ObjectStateStructDat["z"] = timeline.pose.get_value_incremental(t).z
        self.ObjectStateStructDat["h"] = timeline.pose.get_value_incremental(t).h
        self.ObjectStateStructDat["p"] = timeline.pose.get_value_incremental(t).p
        self.ObjectStateStructDat["r"] = timeline.pose.get_value_incremental(t).r
        self.ObjectStateStructDat["roadId"] = timeline.road_id.get_value_incremental(t)
        self.ObjectStateStructDat["laneId"] = timeline.lane_id.get_value_incremental(t)
        self.ObjectStateStructDat["offset"] = timeline.pos_offset.get_value_incremental(t)
        self.ObjectStateStructDat["t"] = timeline.pos_t.get_value_incremental(t)
        self.ObjectStateStructDat["s"] = timeline.pos_s.get_value_incremental(t)

        return self.ObjectStateStructDat

    def build_csv(self):
        if self.extended:
            labels = self.get_labels_line_extended()
        else:
            labels = self.get_labels_line()

        for t in self.timestamps:
            for obj_id in self.objects_timeline.keys():
                state = self.get_object_state_struct_at_time(obj_id, t).copy()
                if not state["active"]:
                    continue
                self.data.append([state[label] for label in labels])

    def fill_timestamps(self):
        filled = []
        i = 0
        curr_time = self.timestamps[0]

        while i < len(self.timestamps) - 1 and curr_time < self.end_time - SMALL_NUMBER:
            filled.append(curr_time)

            next_logged_time = self.timestamps[i + 1]

            if len(self.dt.values) == 2:
                dt = self.dt.values[1][1]
            else:
                dt = self.dt.get_value_binary(curr_time, True)

            next_time = curr_time + dt

            if is_near(next_time, next_logged_time):
                i += 1
                curr_time = next_logged_time
            else:
                prev_dt = dt.get_value_binary(curr_time)
                curr_time += prev_dt

        filled.append(self.timestamps[-1]) # Add last index
        
        self.timestamps = filled

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

    def print_csv(self, extended = False, include_file_refs = True):
        """Print the contents of the .dat file in CSV format to the console."""
        # Print header
        if include_file_refs:
            print(self.get_header_line())

        # Print column headings / value types
        if extended:
            print(', '.join(self.get_labels_line_extended()))
        else:
            print(', '.join(self.get_labels_line()))

        # Read and print all rows of data
        for data in self.data:
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
    dat.print_csv(args.extended, args.file_refs)
    dat.close()
