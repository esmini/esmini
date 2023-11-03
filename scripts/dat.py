import argparse
import ctypes
from enum import Enum
import math
import os

NAME_LEN = 32
VERSION = 2

class CommonPkgHdr(ctypes.Structure):
    _fields_ = [
        ('id', ctypes.c_int),
        ('content_size', ctypes.c_int),
    ]

class CommonPkg():
    def __init__(self):
        self.id = None
        self.content_size = None
        self.content = None

class PkgTime(ctypes.Structure):
    _fields_ = [
        ('time', ctypes.c_double),
    ]

class PkgObjId(ctypes.Structure):
    _fields_ = [
        ('id', ctypes.c_uint),
    ]

class PkgSpeed(ctypes.Structure):
    _fields_ = [
        ('speed', ctypes.c_double),
    ]

class PkgPositions(ctypes.Structure):
    _fields_ = [
        ('x', ctypes.c_double),
        ('y', ctypes.c_double),
        ('z', ctypes.c_double),
        ('h', ctypes.c_double),
        ('p', ctypes.c_double),
        ('r', ctypes.c_double),
    ]

class PkgModelId(ctypes.Structure):
    _fields_ = [
        ('model_id', ctypes.c_int),
    ]

class PkgObjType(ctypes.Structure):
    _fields_ = [
        ('obj_type', ctypes.c_int),
    ]

class PkgObjCategory(ctypes.Structure):
    _fields_ = [
        ('obj_category', ctypes.c_int),
    ]
class PkgCtrlType(ctypes.Structure):
    _fields_ = [
        ('ctrl_type', ctypes.c_int),
    ]

class PkgWheelAngle(ctypes.Structure):
    _fields_ = [
        ('wheel_angle', ctypes.c_double),
    ]

class PkgWheelRot(ctypes.Structure):
    _fields_ = [
        ('wheel_rot', ctypes.c_double),
    ]

class PkgScaleMode(ctypes.Structure):
    _fields_ = [
        ('scale_mode', ctypes.c_int),
    ]

class PkgVisibilityMode(ctypes.Structure):
    _fields_ = [
        ('visibility_mask', ctypes.c_int),
    ]

class PkgRoadId(ctypes.Structure):
    _fields_ = [
        ('road_id', ctypes.c_int),
    ]

class PkgLaneId(ctypes.Structure):
    _fields_ = [
        ('lane_id', ctypes.c_int),
    ]

class PkgPosOffset(ctypes.Structure):
    _fields_ = [
        ('pos_offset', ctypes.c_double),
    ]

class PkgPosT(ctypes.Structure):
    _fields_ = [
        ('pos_T', ctypes.c_double),
    ]

class PkgPosS(ctypes.Structure):
    _fields_ = [
        ('pos_s', ctypes.c_double),
    ]

class PkgBB(ctypes.Structure):
    _fields_ = [
        ('x', ctypes.c_float),
        ('y', ctypes.c_float),
        ('z', ctypes.c_float),
        ('width', ctypes.c_float),
        ('length', ctypes.c_float),
        ('height', ctypes.c_float),
    ]

class LightRGB(ctypes.Structure):
    _fields_ = [
        ('red', ctypes.c_ubyte),
        ('green', ctypes.c_ubyte),
        ('blue', ctypes.c_ubyte),
        ('intensity', ctypes.c_ubyte),

    ]

class PkgLightStates(ctypes.Structure):
    _fields_ = [
        ('day_time_running_lights', LightRGB),
        ('low_beam', LightRGB),
        ('high_beam', LightRGB),
        ('fog_lights_front', LightRGB),
        ('fog_lights_rear', LightRGB),
        ('brake_lights', LightRGB),
        ('indicator_left', LightRGB),
        ('indicator_right', LightRGB),
        ('reversing_lights', LightRGB),
        ('license_plater_illumination', LightRGB),
        ('special_purpose_lights',LightRGB),
        ('fog_lights', LightRGB),
        ('warning_lights', LightRGB),
    ]
    def __getitem__(self, key):
        """
        Accesses elements of the PkgLightStates structure.

        Args:
            key (str or int): The key to access.
                - If str, must be a valid light name (e.g., "low_beam", "fog_lights_front").
                - If int, must be a valid index (0-based) within the structure's fields.

        Returns:
            object: The LightRGB instance representing the accessed element.

        Raises:
            KeyError: If an invalid light name is provided.
            IndexError: If an out-of-range index is used.
        """

        if isinstance(key, str):
            # Access by light name
            try:
                return getattr(self, key)
            except AttributeError:
                raise KeyError(f"Invalid light name: {key}")
        elif isinstance(key, int):
            # Access by index (0-based)
            if 0 <= key < len(self._fields_):
                return getattr(self, self._fields_[key][0])
            else:
                raise IndexError("Index out of range")
        else:
            raise TypeError("Invalid key type")
class PkgId(Enum):
    HEADER      = 11
    TIME_SERIES = 12
    OBJ_ID      = 13
    MODEL_ID    = 14
    POSITIONS   = 15
    SPEED       = 16
    OBJ_TYPE    = 17
    OBJ_CATEGORY = 18
    CTRL_TYPE   = 19
    WHEEL_ANGLE = 20
    WHEEL_ROT   = 21
    BOUNDING_BOX = 22
    SCALE_MODE = 23
    VISIBILITY_MASK = 24
    NAME = 25
    ROAD_ID = 26
    LANE_ID = 27
    POS_OFFSET = 28
    POS_T = 29
    POS_S = 30
    LIGHT_STATES = 31
    OBJ_DELETED = 32
    OBJ_ADDED = 33
    END_OF_SCENARIO = 34

class Mode(Enum):
    ORIGINAL = 0
    MIN_STEP = 1
    MIN_STEP_MIXED = 2
    CUSTOM_TIME_STEP = 3
    CUSTOM_TIME_STEP_MIXED = 4

class objectState():
    def __init__(self):
        self.obj_id = None
        self.obj_active = None
        self.model_id = None
        self.speed = None
        self.pos = None
        self.obj_type = None
        self.obj_category = None
        self.ctrl_type = None
        self.wheel_angle = None
        self.wheel_rot = None
        self.bb = None
        self.scale_mode = None
        self.visibility_mask = None
        self.name = None
        self.road_id = None
        self.lane_id = None
        self.pos_offset = None
        self.pos_T = None
        self.pos_S = None
        self.light_states = None


class CompleteObjectState():
    def __init__(self):
        self.time = None
        self.objectState_ = []

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

        self.filename = filename
        self.version = []
        self.odr_filename = []
        self.mdl_filename = []
        self.delta_time = 1e+10
        self.pkgs = []
        self.get_all_pkg()
        self.start_time = []
        self.index = 0
        self.stop_time = []
        self.fill_time_details()
        self.time = self.start_time
        self.CompleteObjectState_ = CompleteObjectState()
        self.InitiateStates()

        if (self.version.value != VERSION):
            print('Version mismatch. {} is version {} while supported version is: {}'.format(
                filename, self.version.value, VERSION)
            )
            exit(-1)

    def fill_time_details(self):
        for pkg in self.pkgs:
            if pkg.id == PkgId.TIME_SERIES.value:
                self.start_time = pkg.content.time
                break
        for pkg in reversed(self.pkgs):
            if pkg.id == PkgId.TIME_SERIES.value:
                self.stop_time = pkg.content.time
                break

    def get_header_line(self):
        return 'Version: {}, OpenDRIVE: {}, 3DModel: {}'.format(
                self.version.value,
                self.odr_filename,
                self.mdl_filename,
            )
    def get_all_pkg(self):
        stat = os.stat(self.file.name)
        previousTime_ = float("nan")
        while True:
            if self.file.tell() == stat.st_size:
                break # reach end of file
            header_buffer = self.file.read(ctypes.sizeof(CommonPkgHdr))

            header = CommonPkgHdr.from_buffer_copy(header_buffer)
            pkg = CommonPkg()
            pkg.id = header.id
            pkg.content_size = header.content_size

            if header.id == PkgId.HEADER.value:
                version_buffer = self.file.read(ctypes.sizeof(ctypes.c_int))
                self.version = ctypes.c_int.from_buffer_copy(version_buffer)
                odr_size_buffer = self.file.read(ctypes.sizeof(ctypes.c_int))
                odr_size = ctypes.c_int.from_buffer_copy(odr_size_buffer)
                odr_filename_bytes = self.file.read(odr_size.value)
                self.odr_filename = odr_filename_bytes[:odr_size.value].decode('utf-8')
                if isinstance(self.odr_filename, str) and len(self.odr_filename) != 0: # remove white space
                    self.odr_filename = self.odr_filename.rstrip('\x00')

                mdl_size_buffer = self.file.read(ctypes.sizeof(ctypes.c_int))
                mdl_size = ctypes.c_int.from_buffer_copy(mdl_size_buffer)
                mdl_filename_bytes = self.file.read(mdl_size.value)
                self.mdl_filename = mdl_filename_bytes[:mdl_size.value].decode('utf-8')
                if isinstance(self.mdl_filename, str) and len(self.mdl_filename) != 0: # remove white space
                    self.mdl_filename = self.mdl_filename.rstrip('\x00')

            elif header.id == PkgId.TIME_SERIES.value:
                time_buffer = self.file.read(header.content_size)
                t = PkgTime.from_buffer_copy(time_buffer)
                pkg.content = t
                self.pkgs.append(pkg)
                if not (t.time < 0) or not abs(self.delta_time - 1e+10) < 1e-6:
                    if not math.isnan(previousTime_):
                        if abs(t.time - previousTime_) < self.delta_time:
                            self.delta_time = abs(t.time - previousTime_)

                    previousTime_ = t.time

            elif header.id == PkgId.OBJ_ID.value:
                obj_id_buffer = self.file.read(header.content_size)
                obj_id = PkgObjId.from_buffer_copy(obj_id_buffer)
                pkg.content = obj_id
                self.pkgs.append(pkg)
            elif header.id == PkgId.OBJ_ADDED.value:
                self.pkgs.append(pkg) # object added
            elif header.id == PkgId.OBJ_DELETED.value:
                self.pkgs.append(pkg) # object deleted
            elif header.id == PkgId.SPEED.value:
                speed_buffer = self.file.read(header.content_size)
                speed = PkgSpeed.from_buffer_copy(speed_buffer)
                pkg.content = speed
                self.pkgs.append(pkg)
            elif header.id == PkgId.POSITIONS.value:
                pos_buffer = self.file.read(header.content_size)
                pos = PkgPositions.from_buffer_copy(pos_buffer)
                pkg.content = pos
                self.pkgs.append(pkg)
            elif header.id == PkgId.NAME.value:
                name_buffer = self.file.read(header.content_size)
                name = name_buffer[:header.content_size].decode('utf-8')
                if isinstance(name, str) and len(name) != 0: # remove white space
                    name = name.rstrip('\x00')
                pkg.content = name
                self.pkgs.append(pkg)
            elif header.id == PkgId.MODEL_ID.value:
                model_id_buffer = self.file.read(header.content_size)
                model_id = PkgModelId.from_buffer_copy(model_id_buffer)
                pkg.content = model_id
                self.pkgs.append(pkg)
            elif header.id == PkgId.OBJ_TYPE.value:
                obj_type_buffer = self.file.read(header.content_size)
                obj_type = PkgObjType.from_buffer_copy(obj_type_buffer)
                pkg.content = obj_type
                self.pkgs.append(pkg)
            elif header.id == PkgId.OBJ_CATEGORY.value:
                obj_category_buffer = self.file.read(header.content_size)
                obj_category = PkgObjCategory.from_buffer_copy(obj_category_buffer)
                pkg.content = obj_category
                self.pkgs.append(pkg)
            elif header.id == PkgId.CTRL_TYPE.value:
                ctrl_type_buffer = self.file.read(header.content_size)
                ctrl_type = PkgCtrlType.from_buffer_copy(ctrl_type_buffer)
                pkg.content = ctrl_type
                self.pkgs.append(pkg)
            elif header.id == PkgId.WHEEL_ANGLE.value:
                wheel_angle_buffer = self.file.read(header.content_size)
                wheel_angle = PkgWheelAngle.from_buffer_copy(wheel_angle_buffer)
                pkg.content = wheel_angle
                self.pkgs.append(pkg)
            elif header.id == PkgId.WHEEL_ROT.value:
                wheel_rot_buffer = self.file.read(header.content_size)
                wheel_rot = PkgWheelRot.from_buffer_copy(wheel_rot_buffer)
                pkg.content = wheel_rot
                self.pkgs.append(pkg)
            elif header.id == PkgId.BOUNDING_BOX.value:
                bb_buffer = self.file.read(header.content_size)
                bb = PkgBB.from_buffer_copy(bb_buffer)
                pkg.content = bb
                self.pkgs.append(pkg)
            elif header.id == PkgId.SCALE_MODE.value:
                scale_mode_buffer = self.file.read(header.content_size)
                scale_mode = PkgScaleMode.from_buffer_copy(scale_mode_buffer)
                pkg.content = scale_mode
                self.pkgs.append(pkg)
            elif header.id == PkgId.VISIBILITY_MASK.value:
                visibility_mask_buffer = self.file.read(header.content_size)
                visibility_mask = PkgVisibilityMode.from_buffer_copy(visibility_mask_buffer)
                pkg.content = visibility_mask
                self.pkgs.append(pkg)
            elif header.id == PkgId.ROAD_ID.value:
                road_id_buffer = self.file.read(header.content_size)
                road_id = PkgRoadId.from_buffer_copy(road_id_buffer)
                pkg.content = road_id
                self.pkgs.append(pkg)
            elif header.id == PkgId.LANE_ID.value:
                lane_id_buffer = self.file.read(header.content_size)
                lane_id = PkgLaneId.from_buffer_copy(lane_id_buffer)
                pkg.content = lane_id
                self.pkgs.append(pkg)
            elif header.id == PkgId.POS_OFFSET.value:
                pos_offset_buffer = self.file.read(header.content_size)
                pos_offset = PkgPosOffset.from_buffer_copy(pos_offset_buffer)
                pkg.content = pos_offset
                self.pkgs.append(pkg)
            elif header.id == PkgId.POS_S.value:
                pos_s_buffer = self.file.read(header.content_size)
                pos_s = PkgPosS.from_buffer_copy(pos_s_buffer)
                pkg.content = pos_s
                self.pkgs.append(pkg)
            elif header.id == PkgId.POS_T.value:
                pos_t_buffer = self.file.read(header.content_size)
                pos_t = PkgPosT.from_buffer_copy(pos_t_buffer)
                pkg.content = pos_t
                self.pkgs.append(pkg)
            elif header.id == PkgId.LIGHT_STATES.value:
                light_buffer = self.file.read(header.content_size)
                light_states =PkgLightStates.from_buffer_copy(light_buffer)
                pkg.content = light_states
                self.pkgs.append(pkg)
            elif header.id == PkgId.END_OF_SCENARIO.value:
                self.pkgs.append(pkg) # end of scenario
            else:
                print("unknown pkg")
                # ignore_buffer = self.file.read(header.content_size) # ignore it
                raise IndexError(f"Unknown pkg detected:{header.id}")

        self.file.close()

    def InitiateStates(self):
        first_time_frame = False
        new_obj = False
        objectState_ = objectState()
        for pkg in self.pkgs:
            if pkg.id == PkgId.TIME_SERIES.value:
                if first_time_frame == True:
                    break
                self.CompleteObjectState_.time = pkg.content.time
                first_time_frame = True
            elif pkg.id == PkgId.OBJ_ID.value:
                if new_obj == True:
                    # print("object id added->", pkg.content.id)
                    self.CompleteObjectState_.objectState_ .append(objectState_) # append for each object
                    objectState_ = objectState()
                    new_obj = False
                objectState_.obj_id = pkg.content
                new_obj = True
            elif pkg.id == PkgId.OBJ_ADDED.value:
                objectState_.obj_active = True
            elif pkg.id == PkgId.OBJ_DELETED.value:
                objectState_.obj_active = False
            elif pkg.id == PkgId.MODEL_ID.value:
                objectState_.model_id =pkg.content
            elif pkg.id == PkgId.SPEED.value:
                objectState_.speed = pkg.content
            elif pkg.id == PkgId.POSITIONS.value:
                objectState_.pos = pkg.content
            elif pkg.id == PkgId.OBJ_TYPE.value:
                objectState_.obj_type = pkg.content
            elif pkg.id == PkgId.OBJ_CATEGORY.value:
                objectState_.obj_category = pkg.content
            elif pkg.id == PkgId.CTRL_TYPE.value:
                objectState_.ctrl_type = pkg.content
            elif pkg.id == PkgId.WHEEL_ANGLE.value:
                objectState_.wheel_angle = pkg.content
            elif pkg.id == PkgId.WHEEL_ROT.value:
                objectState_.wheel_rot = pkg.content
            elif pkg.id == PkgId.BOUNDING_BOX.value:
                objectState_.bb = pkg.content
            elif pkg.id == PkgId.SCALE_MODE.value:
                objectState_.scale_mode = pkg.content
            elif pkg.id == PkgId.VISIBILITY_MASK.value:
                objectState_.visibility_mask = pkg.content
            elif pkg.id == PkgId.NAME.value:
                objectState_.name = pkg.content
            elif pkg.id == PkgId.ROAD_ID.value:
                objectState_.road_id = pkg.content
            elif pkg.id == PkgId.LANE_ID.value:
                objectState_.lane_id = pkg.content
            elif pkg.id == PkgId.POS_OFFSET.value:
                objectState_.pos_offset = pkg.content
            elif pkg.id == PkgId.POS_T.value:
                objectState_.pos_T = pkg.content
            elif pkg.id == PkgId.POS_S.value:
                objectState_.pos_S = pkg.content
            elif pkg.id == PkgId.LIGHT_STATES.value:
                objectState_.light_states = pkg.content
        self.CompleteObjectState_.objectState_ .append(objectState_)
        # print("object id added->", objectState_.obj_id.id)

    def get_labels_line_extended(self):
        return 'time, id, name, x, y, z, h, p, r, roadId, laneId, offset, t, s, \
speed, wheel_angle, wheel_rot, day_light, low_beam, high_beam, fog_light_front, fog_light_rear, brake_light, \
ind_left, ind_right, reversing_light, license_plate, special_pur_light, fog_light, warning_light'

    def get_labels_line(self):
        return 'time, id, name, x, y, z, h, p, r, speed, wheel_angle, wheel_rot'

    def save_csv(self, include_file_refs = False , extended = False, mode = "original", step_time = 0.05):

        # print("processing mode:", mode)
        csvfile = os.path.splitext(self.filename)[0] + '.csv'
        try:
            fcsv = open(csvfile, 'w')
        except OSError:
            print('ERROR: Could not open file {} for writing'.format(csvfile))
            raise

        # Save column headings / value types
        if include_file_refs:
            fcsv.write(self.get_header_line() + '\n')

        if extended:
            fcsv.write(self.get_labels_line_extended() + '\n')
        else:
            fcsv.write(self.get_labels_line() + '\n')


        if mode == "original":
            mode_ = Mode.ORIGINAL
        elif mode == "min_step":
            mode_ = Mode.MIN_STEP
        elif mode == "min_step_mixed":
            mode_ = Mode.MIN_STEP_MIXED
        elif mode == "time_step":
            mode_ = Mode.CUSTOM_TIME_STEP
        elif mode == "time_step_mixed":
            mode_ = Mode.CUSTOM_TIME_STEP_MIXED

        if( mode_ == Mode.ORIGINAL):
            while(True):
                for state in self.CompleteObjectState_.objectState_:
                    if state.obj_active is True: # only write for active objects. may be it deleted
                        if extended:
                            data = '{:.3f}, {}, {}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {}, {}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, '.format(
                                    self.CompleteObjectState_.time,
                                    state.obj_id.id,
                                    state.name,
                                    state.pos.x,
                                    state.pos.y,
                                    state.pos.z,
                                    state.pos.h,
                                    state.pos.p,
                                    state.pos.r,
                                    state.road_id.road_id,
                                    state.lane_id.lane_id,
                                    state.pos_offset.pos_offset,
                                    state.pos_T.pos_T,
                                    state.pos_S.pos_s,
                                    state.speed.speed,
                                    state.wheel_angle.wheel_angle,
                                    state.wheel_rot.wheel_rot)
                            light_state = PkgLightStates()
                            light_state = state.light_states
                            for i in range(len(PkgLightStates._fields_)):
                                data += '#{:02X}{:02X}{:02X}-{:02X}, '.format(
                                        light_state[i].red, light_state[i].green, light_state[i].blue, light_state[i].intensity)

                        else:
                            data = '{:.3f}, {}, {}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}'.format(
                                    self.CompleteObjectState_.time,
                                    state.obj_id.id,
                                    state.name,
                                    state.pos.x,
                                    state.pos.y,
                                    state.pos.z,
                                    state.pos.h,
                                    state.pos.p,
                                    state.pos.r,
                                    state.speed.speed,
                                    state.wheel_angle.wheel_angle,
                                    state.wheel_rot.wheel_rot)
                        fcsv.write(data + '\n')
                if abs(self.CompleteObjectState_.time - self.stop_time) < 1e-6:
                    break
                self.moveToNextTime()
                self.CompleteObjectState_.time = self.time
                self.updateCache()
        else:
            perviousTimeToMove = 1e-6
            stopAtEachFrame = False
            while(True):

                if mode_ == Mode.MIN_STEP or mode_ == Mode.MIN_STEP_MIXED:
                    timeToMove = self.CompleteObjectState_.time + self.delta_time
                else:
                    timeToMove = self.CompleteObjectState_.time + step_time

                if timeToMove > self.stop_time: # set last time stamp
                    timeToMove = self.stop_time

                if mode_ == Mode.CUSTOM_TIME_STEP_MIXED or mode_ == Mode.MIN_STEP_MIXED:
                    stopAtEachFrame = True
                    if ((self.IsEqual(perviousTimeToMove, self.CompleteObjectState_.time)) is not True and self.IsEqual(self.CompleteObjectState_.time, self.start_time) is not True): # use pervious time till it reaches, ignore start time
                        timeToMove = perviousTimeToMove

                for state in self.CompleteObjectState_.objectState_:
                    if state.obj_active is True: # only write for active objects. may be it deleted
                        if extended:
                            data = '{:.3f}, {}, {}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {}, {}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}'.format(
                                    self.CompleteObjectState_.time,
                                    state.obj_id.id,
                                    state.name,
                                    state.pos.x,
                                    state.pos.y,
                                    state.pos.z,
                                    state.pos.h,
                                    state.pos.p,
                                    state.pos.r,
                                    state.road_id.road_id,
                                    state.lane_id.lane_id,
                                    state.pos_offset.pos_offset,
                                    state.pos_T.pos_T,
                                    state.pos_S.pos_s,
                                    state.speed.speed,
                                    state.wheel_angle.wheel_angle,
                                    state.wheel_rot.wheel_rot)

                            light_state = PkgLightStates()
                            light_state = state.light_states
                            for i in range(len(PkgLightStates._fields_)):
                                data += '#{:02X}{:02X}{:02X}-{:02X},'.format(
                                        light_state[i].red, light_state[i].green, light_state[i].blue, light_state[i].intensity)
                        else:
                            data = '{:.3f}, {}, {}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}'.format(
                                    self.CompleteObjectState_.time,
                                    state.obj_id.id,
                                    state.name,
                                    state.pos.x,
                                    state.pos.y,
                                    state.pos.z,
                                    state.pos.h,
                                    state.pos.p,
                                    state.pos.r,
                                    state.speed.speed,
                                    state.wheel_angle.wheel_angle,
                                    state.wheel_rot.wheel_rot)
                        fcsv.write(data + '\n')
                if self.IsEqual(self.CompleteObjectState_.time, self.stop_time):
                    break
                self.MoveToTime(timeToMove, stopAtEachFrame)
                perviousTimeToMove = timeToMove

        fcsv.close()

    def moveToNextTime(self):
        for i in range(self.index + 1, len(self.pkgs)):
            if self.pkgs[i].id == PkgId.TIME_SERIES.value:
                self.time = self.pkgs[i].content.time
                self.index = i
                break

    def MoveToTime(self, t, stopAtEachFrame = False):
        while(True):
            if ( self.IsEqual(self.CompleteObjectState_.time, t)): #equal
                break
            else:
                previousTime_ = self.time
                perviousIndex = self.index
                self.moveToNextTime()
                if( self.time > t + 1e-6): # gone past time
                    self.time = previousTime_ # set the pervious time and index
                    self.index = perviousIndex
                    self.CompleteObjectState_.time = t
                    break
                else:
                    self.CompleteObjectState_.time = self.time
                    self.updateCache()
                    if stopAtEachFrame:
                        break

    def IsEqual(self, a, b):
        return (abs(a - b) < 1e-6)

    def updateCache(self):
        for i in range(self.index + 1, len(self.pkgs)):
            if self.pkgs[i].id == PkgId.TIME_SERIES.value:
                break # next time frame, stop it
            if self.pkgs[i].id == PkgId.OBJ_ID.value:
                obj_id = self.pkgs[i].content
                continue
            if self.pkgs[i].id == PkgId.OBJ_ADDED.value: # new object added dynamically
                objectState_ = objectState()
                objectState_.obj_id = obj_id
                objectState_.obj_active = True
                self.CompleteObjectState_.objectState_ .append(objectState_)
                # print("object id added->", obj_id.id)
                continue
            if self.pkgs[i].id == PkgId.END_OF_SCENARIO.value:
                break
            for j in range(len(self.CompleteObjectState_.objectState_)):
                if self.CompleteObjectState_.objectState_[j].obj_id.id != obj_id.id:
                    continue    # obj id matched
                elif self.pkgs[i].id == PkgId.OBJ_DELETED.value:
                    self.CompleteObjectState_.objectState_[j].obj_active = False
                    # print("object id-> deleted", self.CompleteObjectState_.objectState_[j].obj_id.id)
                    self.CompleteObjectState_.objectState_.remove(self.CompleteObjectState_.objectState_[j]) # remove it for safe side
                    break
                elif self.pkgs[i].id == PkgId.MODEL_ID.value:
                    self.CompleteObjectState_.objectState_[j].model_id =self.pkgs[i].content
                elif self.pkgs[i].id == PkgId.SPEED.value:
                    self.CompleteObjectState_.objectState_[j].speed = self.pkgs[i].content
                elif self.pkgs[i].id == PkgId.POSITIONS.value:
                    self.CompleteObjectState_.objectState_[j].pos = self.pkgs[i].content
                elif self.pkgs[i].id == PkgId.OBJ_TYPE.value:
                    self.CompleteObjectState_.objectState_[j].obj_type = self.pkgs[i].content
                elif self.pkgs[i].id == PkgId.OBJ_CATEGORY.value:
                    self.CompleteObjectState_.objectState_[j].obj_category = self.pkgs[i].content
                elif self.pkgs[i].id == PkgId.CTRL_TYPE.value:
                    self.CompleteObjectState_.objectState_[j].ctrl_type = self.pkgs[i].content
                elif self.pkgs[i].id == PkgId.WHEEL_ANGLE.value:
                    self.CompleteObjectState_.objectState_[j].wheel_angle = self.pkgs[i].content
                elif self.pkgs[i].id == PkgId.WHEEL_ROT.value:
                    self.CompleteObjectState_.objectState_[j].wheel_rot = self.pkgs[i].content
                elif self.pkgs[i].id == PkgId.BOUNDING_BOX.value:
                    self.CompleteObjectState_.objectState_[j].bb = self.pkgs[i].content
                elif self.pkgs[i].id == PkgId.SCALE_MODE.value:
                    self.CompleteObjectState_.objectState_[j].scale_mode = self.pkgs[i].content
                elif self.pkgs[i].id == PkgId.VISIBILITY_MASK.value:
                    self.CompleteObjectState_.objectState_[j].visibility_mask = self.pkgs[i].content
                elif self.pkgs[i].id == PkgId.NAME.value:
                    self.CompleteObjectState_.objectState_[j].name = self.pkgs[i].content
                elif self.pkgs[i].id == PkgId.ROAD_ID.value:
                    self.CompleteObjectState_.objectState_[j].road_id = self.pkgs[i].content
                elif self.pkgs[i].id == PkgId.LANE_ID.value:
                    self.CompleteObjectState_.objectState_[j].lane_id = self.pkgs[i].content
                elif self.pkgs[i].id == PkgId.POS_OFFSET.value:
                    self.CompleteObjectState_.objectState_[j].pos_offset = self.pkgs[i].content
                elif self.pkgs[i].id == PkgId.POS_T.value:
                    self.CompleteObjectState_.objectState_[j].pos_T = self.pkgs[i].content
                elif self.pkgs[i].id == PkgId.POS_S.value:
                    self.CompleteObjectState_.objectState_[j].pos_S = self.pkgs[i].content
                elif self.pkgs[i].id == PkgId.LIGHT_STATES.value:
                    self.CompleteObjectState_.objectState_[j].light_states = self.pkgs[i].content


if __name__ == "__main__":
    # Create the parser
    parser = argparse.ArgumentParser(description='Read and print .dat file')

    parser.add_argument('filename', help='dat filename')
    parser.add_argument('--extended', '-e', action='store_true', help='add road coordinates')
    parser.add_argument('--file_refs', '-r', action='store_true', help='include odr and model file references')
    parser.add_argument("--time_mode", "-m", choices=["original", "min_step", "min_step_mixed", "custom_time_step", "custom_time_step_mixed"], default="original", help="control timestamps in the csv.")
    parser.add_argument(
        "-t",
        "--time_step",
        type=float,
        action="store",
        default=0.05,
        help="The time step to use for the fixed time steps(ms).",
    )


    # Execute the parse_args() method
    args = parser.parse_args()

    dat = DATFile(args.filename)
    dat.save_csv(extended = True if args.extended else False, include_file_refs=True if args.file_refs else False, mode = args.time_mode, step_time =args.time_step)

    # dat = DATFile('light_test_simple11.dat')
    # dat.save_csv(extended = True)
