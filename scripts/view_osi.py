import matplotlib.pyplot as plt
import matplotlib.animation as anim
import matplotlib.collections as mc
import numpy as np
import os
import sys
import struct
import ctypes as ct

# Add scripts root directory to module search path in order to find osi3
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../', 'scripts')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'scripts')))  # if script moved to Hello World folder

from osi3.osi_groundtruth_pb2 import GroundTruth

# Import esmini lib
if sys.platform == "linux" or sys.platform == "linux2":
    se = ct.CDLL("../bin/libesminiLib.so")
elif sys.platform == "darwin":
    se = ct.CDLL("../bin/libesminiLib.dylib")
elif sys.platform == "win32":
    se = ct.CDLL("../bin/esminiLib.dll")
else:
    print("Unsupported platform: {}".format(platform))
    quit()


# Specify argument and return types of some esmini functions
se.SE_GetOSIGroundTruth.restype = ct.c_void_p
se.SE_GetOSIGroundTruth.argtypes = [ct.c_void_p]
se.SE_StepDT.argtypes = [ct.c_float]

class View:
    def __init__(self):
        # Create the figure and axis
        self.fig, self.ax = plt.subplots()
        self.canvas = self.fig.canvas

        # Static content: Labels, grids, etc.
        self.ax.set_title('OSI plot')
        self.ax.set_xlabel('X-axis')
        self.ax.set_ylabel('Y-axis')
        self.ax.grid(True)
        self.ax.set_aspect('equal', 'datalim')
        self.canvas.mpl_connect('resize_event', self.on_resize)

    # Adjust the axis margins (padding between the axis and the figure borders)
    def adjust_margins(self):
        self.screen_width, self.screen_height = self.canvas.get_width_height()
        self.fig.subplots_adjust(left=70.0/self.screen_width,
                                 right=1.0-15.0/self.screen_width,
                                 top=1.0-25/self.screen_height,
                                 bottom=45/self.screen_height,)

    # Resize event callback
    def on_resize(self, event):
        self.adjust_margins()

    def rmtype2string(self, type):
        if type == 0: return 'UNKNOWN'
        elif type == 1: return 'OTHER'
        elif type == 2: return 'NO_LINE'
        elif type == 3: return 'SOLID_LINE'
        elif type == 4: return 'DASHED_LINE'
        elif type == 5: return 'BOTTS_DOTS'
        elif type == 6: return 'ROAD_EDGE'
        elif type == 7: return 'SNOW_EDGE'
        elif type == 8: return 'GRASS_EDGE'
        elif type == 9: return 'GRAVEL_EDGE'
        elif type == 10: return 'SOIL_EDGE'
        elif type == 11: return 'GUARD_RAIL'
        elif type == 12: return 'CURB'
        elif type == 13: return 'STRUCTURE'
        elif type == 14: return 'BARRIER'
        elif type == 15: return 'SOUND_BARRIER'
        else: return 'UNSUPPORTED: {}'.format(type)

    def add_static_content(self, gt):
        # lines = []
        # for lane in gt.lane:
        #     clf=lane.classification
        #     i=0
        #     line=[]
        #     while i < len(clf.centerline):
        #         p0 = clf.centerline[i]
        #         line += [(p0.x, p0.y)]
        #         i += 1
        #     lines.append(line)
        # lc = mc.LineCollection(lines, color='#AAAAAA')
        # self.ax.add_collection(lc)

        legends = []
        legend_strings = []
        for l in gt.lane_boundary:
            i=0
            type = l.classification.type
            if type not in legends:
                legends.append(type)
                legend_strings.append(self.rmtype2string(type))
            if type == 4:  # dashed line
                points = []  # create one list for polyline
            else:
                points = [[],[]]  # create lists for x and y data separately, for solid line plots

            if type == 3 or type == 4 or type == 5:  # solid line or dashed line or botts dots
                color = '#222222'
            else:
                if type == 2:
                    print('Boundary type {} plotted with light gray color'.format(self.rmtype2string(type)))
                    color = '#EEEEEE'  # light yellow for NO_LINE
                else:
                    print('Unsupported lane boundary type: {} plotted with light red color'.format(self.rmtype2string(type)))
                    color = '#FFAAAA'  # light gray as default for various unsupported line types

            while i < len(l.boundary_line):
                if type == 4:  # dashed line
                    p0 = l.boundary_line[i].position
                    if i < len(l.boundary_line)-1:
                        p1 = l.boundary_line[i+1].position
                    else:
                        p1 = l.boundary_line[i].position  # make a dot instead of line
                    points.append([(p0.x, p0.y), (p1.x, p1.y)])
                    i += 2
                else:
                    p = l.boundary_line[i].position
                    points[0].append(p.x)
                    points[1].append(p.y)
                    i += 1
            if type == 4:  # dashed line
                lc = mc.LineCollection(points, color=color)
                self.ax.add_collection(lc)
            else:  # solid line
                self.ax.plot(points[0], points[1], color=color)
        self.ax.legend(legend_strings)

        # for i, rm in enumerate(gt.road_marking):
        #     self.x_roadmark = []
        #     self.y_roadmark = []
        #     bps = rm.base.base_polygon
        #     for bp in bps:
        #         self.x_roadmark.append(bps.x)
        #         self.y_roadmark.append(bps.y)
        #     self.ax.plot(self.x_roadmark, self.y_roadmark, color='#333333', label='LaneBoundary' if i==0 else '')

    def add_dynamic_content(self):
        # Dynamic content: Plot a dynamic line (that will update)
        self.dynamic_line, = self.ax.plot([], [], color='red', label='Dynamic Line', linewidth=2)

        # Set x and y limits for the dynamic line
        self.ax.set
        # self.ax.set_xlim(0, 2*np.pi)
        # self.ax.set_ylim(-1.5, 1.5)

        # Show legend
        # self.ax.legend()

    # Function to initialize the dynamic plot (blank at the beginning)
    def init(self):
        self.dynamic_line.set_data([], [])
        return self.dynamic_line,

    # Function to update the dynamic plot (called repeatedly)
    def update(self, frame):
        self.x_dynamic = np.linspace(0, 2*np.pi, 100)
        self.y_dynamic = np.sin(self.x_dynamic + frame / 10.0)  # Dynamic change
        self.dynamic_line.set_data(self.x_dynamic, self.y_dynamic)
        return self.dynamic_line,

    def render(self):
        # Blitting is enabled here to prevent full figure redraw
        ani = anim.FuncAnimation(self.fig, self.update, frames=range(100), init_func=self.init, blit=True)
        plt.show()

class OSIFile:
    def __init__(self, osi_filename):
        try:
            self.file = open(osi_filename, 'rb')
        except OSError:
            print('ERROR: Could not open file {} for reading'.format(osi_filename))
            raise

        self.filename = osi_filename
        self.gt = GroundTruth()
        self.read_next_message()

        self.view = View()
        self.view.add_static_content(self.gt)
        self.view.add_dynamic_content()

    def close(self):
        self.file.close()

    def read_next_message(self):
        size_bin = self.file.read(4)
        if len(size_bin) < 4:
            return False
        else:
            msg_size = struct.unpack('I', size_bin)[0]
            msg = self.file.read(msg_size)
            self.gt.ParseFromString(msg)
        return True

    def run(self):
        self.view.render()

class Sim:
    def __init__(self, scenario):
        self.gt = GroundTruth()  # OSI groundtruth message
        self.gt_size = ct.c_int()
        if se.SE_Init(scenario, 0, 0, 0, 0) != 0:
            print('Failed to open ', scenario)
            exit(-1)

        se.SE_UpdateOSIGroundTruth()
        gt_string = se.SE_GetOSIGroundTruth(ct.byref(self.gt_size))
        self.gt.ParseFromString(ct.string_at(gt_string, self.gt_size))

        self.view = View()
        self.view.add_static_content(self.gt)
        self.view.add_dynamic_content()
        # self.GetAndPrintOSIInfo()  # Initial state

    # Retrieve OSI message and print some info
    def GetAndPrintOSIInfo(self):
        se.SE_UpdateOSIGroundTruth()

        msg_string = se.SE_GetOSIGroundTruth(ct.byref(self.msg_size))
        self.msg.ParseFromString(ct.string_at(msg_string, self.msg_size))

        # Print some info
        print("Timestamp: {:.2f}".format(self.msg.timestamp.seconds + 1e-9 * self.msg.timestamp.nanos))
        for j, o in enumerate(self.msg.moving_object):
            print('   Object[{}] id {}'.format(j, o.id.value))
            print('      pos.x {:.2f} pos.y {:.2f} rot.h {:.2f}'.format(\
                o.base.position.x,\
                o.base.position.y,\
                o.base.orientation.yaw))
            print('      vel.x {:.2f} vel.y {:.2f} rot_rate.h {:.2f}'.format(\
                o.base.velocity.x,\
                o.base.velocity.y,\
                o.base.orientation_rate.yaw))
            print('      acc.x {:.2f} acc.y {:.2f} rot_acc.h {:.2f}'.format(\
                o.base.acceleration.x,\
                o.base.acceleration.y,\
                o.base.orientation_acceleration.yaw))


    def run(self):
        self.view.render()
        # while se.SE_GetQuitFlag() != 1:
        #     se.SE_StepDT(0.05)
        #     self.view.render()
            # self.GetAndPrintOSIInfo()  # Updated state

        se.SE_Close()

if __name__ == '__main__':
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        print('Usage: {} <filename>'.format(os.path.basename(sys.argv[0])))
        exit(-1)

    if os.path.splitext(sys.argv[1])[1] == '.xosc':
        sim = Sim(b"../resources/xosc/cut-in.xosc")
        sim.run()
    else:
        osi_file = OSIFile(sys.argv[1])
        osi_file.run()

    exit (0)

