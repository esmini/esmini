import argparse
import struct
import os
import enum
from collections import defaultdict
from typing import List
import copy
import bisect
import ctypes

VERSION_MAJOR = 4
VERSION_MINOR = 3
SMALL_NUMBER = 1e-6
LARGE_NUMBER = 1e10


class OldDATHeader(ctypes.Structure):
    """ Structure for old DAT file header."""
    _fields_ = [
        ('version', ctypes.c_int),
        ('odr_filename', ctypes.c_char * 512),
        ('model_filename', ctypes.c_char * 512),
            ]

class PacketId(enum.Enum):
    """
    Enum for packet IDs.

    Shall mirror PacketHandler.hpp Dat::PacketId
    """
    OBJ_ID            = 0
    MODEL_ID          = 1
    OBJ_TYPE          = 2
    OBJ_CATEGORY      = 3
    CTRL_TYPE         = 4
    TIMESTAMP         = 5
    NAME              = 6
    SPEED             = 7
    WHEEL_ANGLE       = 8
    WHEEL_ROT         = 9
    BOUNDING_BOX      = 10
    SCALE_MODE        = 11
    VISIBILITY_MASK   = 12
    POSE              = 13
    ROAD_ID           = 14
    LANE_ID           = 15
    POS_OFFSET        = 16
    POS_T             = 17
    POS_S             = 18
    OBJ_DELETED       = 19
    OBJ_ADDED         = 20
    DT                = 21
    END_OF_SCENARIO   = 22
    TRAFFIC_LIGHT     = 23
    REFPOINT_X_OFFSET = 24
    MODEL_X_OFFSET    = 25
    OBJ_MODEL3D       = 26
    ELEM_STATE_CHANGE = 27
    PACKET_ID_SIZE    = 28

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
    packet_id, data_size = struct.unpack(header_format, header_data)
    return packet_id, data_size

def read_string_packet(file):
    """Read a string packet from the file."""
    size = read_dtype(file, DataType.uint32)
    string_bytes = file.read(size)
    return string_bytes.decode('utf-8')

def is_near(x: float, y: float) -> bool:
    """ Check if two floating point numbers are nearly equal """
    return abs(x - y) < SMALL_NUMBER

class Timeline():
    """
    Contains all the timeline data for a specific variable
    """
    def __init__(self):
        self.values = []
        self.last_index = 0
        self.last_time = LARGE_NUMBER

    def get_value_incremental(self, time: float) -> any:
        """ 
        Get last valid value at time, searching incrementally from last known index
        """
        if len(self.values) == 0:
            return None
        
        if len(self.values) == 1:
            self.last_index = 0
            self.last_time  = self.values[0][0]
            return self.values[0][1]

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

    def get_index_binary(self, time: float) -> int:
        """
        Get index of last valid data at time using binary search
        """
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
        idx = bisect.bisect_right(times, time, lo=search_start)

        return idx

    def get_value_binary(self, time: float, upper: bool = False) -> any:
        """
        Get last valid value at time using binary search.
        If upper is True, get the next value after time if available.
        """
        if len(self.values) == 0:
            print("ERROR, no values available")
            exit(-1)

        if (len(self.values) == 1):
            return self.values[0][1]

        timestamps = [v[0] for v in self.values]
        idx = bisect.bisect_right(timestamps, time)

        if idx == 0:
            return self.values[0][1]
        elif idx == len(self.values):
            return self.values[-1][1]
        
        if not upper:
            idx -= 1

        return self.values[idx][1]



class PropertyTimeline():
    """
    Contains all the timelines and ghost restart time for a specific object
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
        self.refpoint_x_offset = Timeline()
        self.model_x_offset = Timeline()
        self.model3d = Timeline()

class DATFile():
    """
    Functionality to read .dat files and populate the states of all objects over time.
    """
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
        "s": None,
        "refpoint_x_offset": None,
        "model_x_offset": None,
        "model3d": None
    }

    def __init__(self, filename, extended=False):
        self.version_major  = 0
        self.version_minor  = 0
        self.header_size    = 0
        self.odr_filename   = ""
        self.model_filename = ""
        self.git_rev        = ""

        self.check_header(filename)
        self.extended = extended

        self.filename = filename
        self.file_size = os.path.getsize(filename)

        self.labels = self.ObjectStateStructDat.keys()
        self.current_object_id = None
        self.ghost_controller_id = None
        self.ghost_timeline_setup = False
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
    
    def check_header(self, filename: str) -> None:
        """ Check the header of the .dat file and open it for reading. """
        if not os.path.isfile(filename):
            print(f'ERROR: dat-file not found: {filename}')
            return
        try:
            self.file = open(filename, 'rb')
        except OSError:
            print(f'ERROR: Could not open file {filename} for reading')
            return -1

        ret = self.read_dat_header(self.file)

        if ret != 0:
            old_header = self.check_old_header(filename)
            if old_header != -1:
                print(f"ERROR: Old DAT file version {old_header} found which is not supported.")
            else:
                print("ERROR: Unable to read DAT file header, is it really a DAT file?")

            print("ERROR: Failed to read DAT header.")
            exit(-1)

        if self.version_major != VERSION_MAJOR:
            print(f'ERROR: Incompatible DAT major file version: {self.version_major}, supporting: {VERSION_MAJOR}')
            exit(-1)

        if self.version_minor != VERSION_MINOR:
            print(f"Warning: DAT-file has version {self.version_major}.{self.version_minor}. Some inconsistencies are expected.")

    def check_old_header(self, filename: str) -> int:
        """
        Check for old DAT file header.
        """
        try:
            file = open(filename, 'rb')
        except OSError:
            print(f'ERROR: Could not open file {filename} for reading')
            return -1

        header = OldDATHeader.from_buffer_copy(file.read(ctypes.sizeof(OldDATHeader)))
        file.close()
        return header.version

    def read_dat_header(self, file) -> int:
        """Read the header of a .dat file."""
        try:
            self.version_major  = read_dtype(file, DataType.uint32)
            self.version_minor  = read_dtype(file, DataType.uint32)
            self.header_size    = read_dtype(file, DataType.uint32)
            self.odr_filename   = read_string_packet(file)
            self.model_filename = read_string_packet(file)
            self.git_rev        = read_string_packet(file)
            return 0
        except ValueError:
            return -1


    def parse_data(self) -> None:
        """
        Parse the .dat file and extract object states.
        Packets are read in the order they are likely to appear in the file.
        """
        while self.file.tell() < self.file_size:
            p_id, data_size = read_packet_header(self.file)

            # TIMESTAMP packet
            if p_id == PacketId.TIMESTAMP.value:
                self.current_timestamp = read_dtype(self.file, DataType.double)

                if len(self.timestamps) == 0 or self.current_timestamp > self.timestamps[-1]:
                    self.timestamps.append(self.current_timestamp)
                    self.ghost_timeline_setup = False
                elif self.current_timestamp < self.timestamps[-1] and not self.ghost_timeline_setup:
                    self.setup_ghosts_timeline()
                    self.ghost_timeline_setup = True

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
                    self.current_object_timeline = self.objects_timeline[self.current_object_id]

                    if self.current_object_timeline.active.values[-1][1] == False:
                        self.current_object_timeline.active.values.append([self.current_timestamp, True])

            # POSE packet
            elif p_id == PacketId.POSE.value:
                pose = Pose()
                for k in list(pose.__dict__.keys()):
                    setattr(pose, k, read_dtype(self.file, DataType.float))
                self.current_object_timeline.pose.values.append([self.current_timestamp, pose])

            elif p_id == PacketId.DT.value:
                dt = read_dtype(self.file, DataType.double)
                if not is_near(dt, 0.0):
                    self.dt.values.append([self.current_timestamp, dt])
            elif p_id == PacketId.SPEED.value:
                self.current_object_timeline.speed.values.append([self.current_timestamp, read_dtype(self.file, DataType.float)])
            elif p_id == PacketId.WHEEL_ANGLE.value:
                self.current_object_timeline.wheel_angle.values.append([self.current_timestamp, read_dtype(self.file, DataType.float)])
            elif p_id == PacketId.WHEEL_ROT.value:
                self.current_object_timeline.wheel_rot.values.append([self.current_timestamp, read_dtype(self.file, DataType.float)])
            elif p_id == PacketId.POS_OFFSET.value:
                self.current_object_timeline.pos_offset.values.append([self.current_timestamp, read_dtype(self.file, DataType.float)])
            elif p_id == PacketId.POS_T.value:
                self.current_object_timeline.pos_t.values.append([self.current_timestamp, read_dtype(self.file, DataType.float)])
            elif p_id == PacketId.POS_S.value:
                self.current_object_timeline.pos_s.values.append([self.current_timestamp, read_dtype(self.file, DataType.float)])
            elif p_id == PacketId.MODEL_ID.value:
                self.current_object_timeline.model_id.values.append([self.current_timestamp, read_dtype(self.file, DataType.int32)])
            elif p_id == PacketId.OBJ_TYPE.value:
                self.current_object_timeline.obj_type.values.append([self.current_timestamp, read_dtype(self.file, DataType.int32)])
            elif p_id == PacketId.OBJ_CATEGORY.value:
                self.current_object_timeline.obj_category.values.append([self.current_timestamp, read_dtype(self.file, DataType.int32)])
            elif p_id == PacketId.CTRL_TYPE.value:
                ctrl_type = read_dtype(self.file, DataType.int32)
                self.current_object_timeline.ctrl_type.values.append([self.current_timestamp, ctrl_type])
                if ctrl_type == 100:
                    self.ghost_controller_id = self.current_object_id
            elif p_id == PacketId.SCALE_MODE.value:
                self.current_object_timeline.scale_mode.values.append([self.current_timestamp, read_dtype(self.file, DataType.int32)])
            elif p_id == PacketId.VISIBILITY_MASK.value:
                self.current_object_timeline.visibility_mask.values.append([self.current_timestamp, read_dtype(self.file, DataType.int32)])
            elif p_id == PacketId.ROAD_ID.value:
                self.current_object_timeline.road_id.values.append([self.current_timestamp, read_dtype(self.file, DataType.uint32)])
            elif p_id == PacketId.LANE_ID.value:
                self.current_object_timeline.lane_id.values.append([self.current_timestamp, read_dtype(self.file, DataType.int32)])
            elif p_id == PacketId.NAME.value:
                name = read_string_packet(self.file)
                self.current_object_timeline.name.values.append([self.current_timestamp, name])
            elif p_id == PacketId.BOUNDING_BOX.value:
                bb = BoundingBox()
                for k in list(bb.__dict__.keys()):
                    setattr(bb, k, read_dtype(self.file, DataType.float))
                self.current_object_timeline.bounding_box.values.append([self.current_timestamp, bb])
            elif p_id == PacketId.TRAFFIC_LIGHT.value:
                self.file.seek(data_size, 1) # Skip packet, not supported yet
            elif p_id == PacketId.REFPOINT_X_OFFSET.value:
                self.current_object_timeline.refpoint_x_offset.values.append([self.current_timestamp, read_dtype(self.file, DataType.float)])
            elif p_id == PacketId.MODEL_X_OFFSET.value:
                self.current_object_timeline.model_x_offset.values.append([self.current_timestamp, read_dtype(self.file, DataType.float)])
            elif p_id == PacketId.OBJ_MODEL3D.value:
                model3d = read_string_packet(self.file)
                self.current_object_timeline.model3d.values.append([self.current_timestamp, model3d])
            elif p_id == PacketId.ELEM_STATE_CHANGE.value:
                self.file.seek(data_size, 1) # Skip packet, not supported yet

            # OBJ_DELETED packet
            elif p_id == PacketId.OBJ_DELETED.value:
                self.current_object_timeline.active.values.append([self.current_timestamp, False])
            
            # END_OF_SCENARIO packet
            elif p_id == PacketId.END_OF_SCENARIO.value:
                self.end_time = read_dtype(self.file, DataType.double)
                if not is_near(self.end_time, self.timestamps[-1]):
                    self.timestamps.append(self.end_time)

    def setup_ghosts_timeline(self):
        """ Setup timelines for ghost objects upon ghost restart"""
        obj_tl = self.objects_timeline.get(self.ghost_controller_id)

        if obj_tl is None:
            print("ERROR: No ghost controller found even though a ghost restart was detected. Quitting.")
            exit(-1)

        it = self.objects_timeline.get(self.ghost_ghost_counter)

        if it is None:
            self.objects_timeline[self.ghost_ghost_counter] = copy.deepcopy(self.objects_timeline[self.ghost_controller_id])
            it = self.objects_timeline[self.ghost_ghost_counter]

            it.active.values[0][1] = False
            it.name.values[0][1] += f"_{self.ghost_ghost_counter}"
            it.active.values.append([self.current_timestamp, True])
            it.active.values.append([self.timestamps[-1], False])

            self.ghost_ghost_counter -= 1

            obj_tl.last_restart_time = self.current_timestamp

        slice_idx = obj_tl.lane_id.get_index_binary(self.current_timestamp)
        obj_tl.lane_id.values = obj_tl.lane_id.values[0:slice_idx]

        slice_idx = obj_tl.road_id.get_index_binary(self.current_timestamp)
        obj_tl.road_id.values = obj_tl.road_id.values[0:slice_idx]

        slice_idx = obj_tl.pos_offset.get_index_binary(self.current_timestamp)
        obj_tl.pos_offset.values = obj_tl.pos_offset.values[0:slice_idx]

        slice_idx = obj_tl.pos_t.get_index_binary(self.current_timestamp)
        obj_tl.pos_t.values = obj_tl.pos_t.values[0:slice_idx]

        slice_idx = obj_tl.pos_s.get_index_binary(self.current_timestamp)
        obj_tl.pos_s.values = obj_tl.pos_s.values[0:slice_idx]

        slice_idx = obj_tl.pose.get_index_binary(self.current_timestamp)
        obj_tl.pose.values = obj_tl.pose.values[0:slice_idx]

        slice_idx = obj_tl.speed.get_index_binary(self.current_timestamp)
        obj_tl.speed.values = obj_tl.speed.values[0:slice_idx]

        slice_idx = obj_tl.wheel_angle.get_index_binary(self.current_timestamp)
        obj_tl.wheel_angle.values = obj_tl.wheel_angle.values[0:slice_idx]

        slice_idx = obj_tl.wheel_rot.get_index_binary(self.current_timestamp)
        obj_tl.wheel_rot.values = obj_tl.wheel_rot.values[0:slice_idx]

        slice_idx = obj_tl.visibility_mask.get_index_binary(self.current_timestamp)
        obj_tl.visibility_mask.values = obj_tl.visibility_mask.values[0:slice_idx]

        slice_idx = obj_tl.pos_s.get_index_binary(self.current_timestamp)
        obj_tl.pos_s.values = obj_tl.pos_s.values[0:slice_idx]

    def get_object_state_struct_at_time(self, obj_id: int, t: float) -> dict:
        """ Get the state of an object at a specific time."""
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
        self.ObjectStateStructDat["refpoint_x_offset"] = timeline.refpoint_x_offset.get_value_incremental(t)
        self.ObjectStateStructDat["model_x_offset"] = timeline.model_x_offset.get_value_incremental(t)

        return self.ObjectStateStructDat

    def build_data(self, usage: str = 'csv') -> None:
        """
        Build the CSV data from the parsed object states.
        """
        if usage == 'csv':
            if self.extended:
                labels = self.get_labels_line_extended()
            else:
                labels = self.get_labels_line()
        elif usage == 'plot':
            labels = self.labels
        else:
            raise ValueError(f"Unsupported usage type: {usage}, only supported are 'csv' and 'plot'.")

        for t in self.timestamps:
            for obj_id in self.objects_timeline.keys():
                state = self.get_object_state_struct_at_time(obj_id, t).copy()
                if not state["active"]:
                    continue
                self.data.append([state[label] for label in labels])

    def fill_timestamps(self):
        """
        Fill in missing timestamps based on dt values.
        """
        if len(self.dt.values) == 0:
            print("ERROR: No dt values found, cannot fill timestamps.")
            return

        curr_time = self.timestamps[0]
        filled = [curr_time]

        if len(self.dt.values) == 1:
            dt = self.dt.values[0][1]
            self.fill_empty_timestamps(curr_time, self.timestamps[-1], dt, filled)
        else:
            i = 0
            j = 0
            while j < len(self.dt.values) - 1:
                next_timestamp = self.timestamps[i + 1]
                next_dt = self.dt.values[j + 1][1]

                if is_near(curr_time, (self.dt.values[j + 1][0] - next_dt)):
                    j += 1
                elif curr_time - SMALL_NUMBER > self.dt.values[j + 1][0]:
                    start_time = self.dt.values[j + 1][0]
                    end_time = curr_time
                    restart_dt = self.dt.values[j + 1][1]

                    start_idx = bisect.bisect_right(filled, start_time)
                    start_idx -= 1
                    steps = int(round(((end_time - start_time) / restart_dt) + SMALL_NUMBER))
                    for k in range(steps):
                        t = k * restart_dt + start_time
                        for m in range(start_idx, len(filled)):
                            if t > filled[m] + SMALL_NUMBER and t < filled[m + 1] - SMALL_NUMBER:
                                filled.insert(m + 1, t)
                                start_idx = m + 1
                                break
                    j += 1
                    continue

                dt = self.dt.values[j][1]
                if curr_time + next_dt < next_timestamp - SMALL_NUMBER and curr_time + dt < next_timestamp - SMALL_NUMBER:
                    end_time = next_timestamp - next_dt
                    self.fill_empty_timestamps(curr_time, end_time, dt, filled)
                elif j == len(self.dt.values) - 1 and curr_time + dt < self.timestamps[-1] - SMALL_NUMBER:
                    self.fill_empty_timestamps(curr_time, self.timestamps[-1], dt, filled)
                else:
                    filled.append(next_timestamp)
                i += 1
                curr_time = filled[-1]

        self.timestamps = filled
    
    def fill_empty_timestamps(self, start: float, end: float, dt: float, v: List[float]) -> None:
        """ Fill in missing timestamps between start and end using dt """
        steps = int(round(((end - start) / dt) + SMALL_NUMBER))
        for i in range(steps):
            v.append((i + 1) * dt + start)

    def get_header_line(self) -> str:
        """Get the header line with version and file references."""
        return f'Version: {self.version_major}.{self.version_minor}, OpenDRIVE: {self.odr_filename}, 3DModel: {self.model_filename} GIT REV: {self.git_rev}'

    def get_labels_line(self) -> List[str]:
        """ Get the standard labels line """
        return ['time', 'id', 'name', 'x', 'y', 'z', 'h', 'p', 'r', 'speed', 'wheel_angle', 'wheel_rot']

    def get_labels_line_extended(self) -> List[str]:
        """ Get the extended labels line """
        return ['time', 'id', 'name', 'x', 'y', 'z', 'h', 'p', 'r', 'roadId', 'laneId', 'offset', 't', 's', 'speed', 'wheel_angle', 'wheel_rot']

    def get_data_line(self, data) -> str:
        """ Will contain extended data if self.extended is True """
        return ', '.join(f"{x:.3f}" if isinstance(x, float) else str(x) for x in data)


    def print_csv(self, extended = False, include_file_refs = True) -> None:
        """Print the contents of the .dat file in CSV format to the console."""
        # Print header
        self.build_data()
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

    def save_csv(self, extended = False, include_file_refs = True) -> None:
        """Save the contents of the .dat file in CSV format to a file."""
        self.build_data()
        csvfile = os.path.splitext(self.filename)[0] + '.csv'
        try:
            fcsv = open(csvfile, 'w')
        except OSError:
            print(f'ERROR: Could not open file {csvfile} for writing')
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

    def close(self) -> None:
        """Close the .dat file."""
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
