import matplotlib.pyplot as plt
import matplotlib.animation as anim
import matplotlib.collections as mc
import matplotlib.widgets as mw
import matplotlib.lines as lines
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
    def __init__(self, gt):
        # Create the figure and axis
        px = 1/plt.rcParams['figure.dpi']  # pixel size in inches
        self.fig, self.ax = plt.subplots(figsize=(1200*px, 600*px))
        self.canvas = self.fig.canvas
        self.gt = gt
        self.grid_enabled = -1


        # Static content: Labels, grids, etc.
        self.ax.set_title('OSI plot')
        self.ax.set_xlabel('X-axis')
        self.ax.set_ylabel('Y-axis')
        self.ax.grid(True)
        self.ax.set_aspect('equal', 'datalim')
        self.canvas.mpl_connect('resize_event', self.on_resize)

        self.add_static_content(self.gt)
        self.add_dynamic_content()

        self.ani = anim.FuncAnimation(self.fig, self.update, frames=range(100), blit=False, interval=50)

        # Create grid toggle button
        self.gbax = plt.axes([0.05, 0.05, 0.2, 0.075])  # Position for grid button
        self.button_grid = mw.Button(self.gbax, 'Toggle Grid')
        self.button_grid.on_clicked(self.toggle_grid)

        # Create a CheckButtons widget for visibility
        labels = list(self.static_plots.keys())
        self.cbax = plt.axes([1.0, 0.5, 2.0, 0.04*len(labels)])  # Position of the checkbox area

        colors = []
        for l in labels:
            colors.append(self.plot_colors[l])
        self.check = mw.CheckButtons(self.cbax, labels, [True]*len(labels), label_props={'color': colors})
        self.check.on_clicked(self.toggle_visibility)

        self.tax = plt.axes([1.0, 0.5, 2.0, 20])  # Position of the checkbox area
        self.text = mw.TextBox(self.tax, "", "")

        # Define a function to handle the pick event
        def on_pick(event):
            if event.artist == line:
                print(f"Picked point: {event.ind}")

    # Adjust the axis margins (padding between the axis and the figure borders)
    def adjust_margins(self):
        menu_width = 200
        margin = 10
        cb_height = 20*len(self.static_plots.keys())
        self.screen_width, self.screen_height = self.canvas.get_width_height()
        y = 25
        self.fig.subplots_adjust(left=70.0/self.screen_width,
                                 right=1.0-(2*margin + menu_width)/self.screen_width,
                                 top=1.0-y/self.screen_height,
                                 bottom=45/self.screen_height,)
        y += cb_height
        self.cbax.set_position([1.0-(margin + menu_width)/self.screen_width,
                               1-y/self.screen_height,
                               menu_width/self.screen_width,
                               cb_height/self.screen_height])
        y += margin + 30
        self.gbax.set_position([1.0-(margin + menu_width)/self.screen_width,
                               1-y/self.screen_height,
                               menu_width/self.screen_width,
                               30/self.screen_height])
        y += margin + 30
        self.tax.set_position([1.0-(margin + menu_width)/self.screen_width,
                               1-y/self.screen_height,
                               menu_width/self.screen_width,
                               30/self.screen_height])

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
        self.static_plots = {}
        self.plot_colors = {}
        lines = []
        for lane in gt.lane:
            clf=lane.classification
            i=0
            line=[]
            while i < len(clf.centerline):
                p0 = clf.centerline[i]
                line += [(p0.x, p0.y)]
                i += 1
            lines.append(line)

        self.plot_colors["CenterLine"] = '#BBBBFF'
        lc = mc.LineCollection(lines, picker=5, label="CenterLine", color=self.plot_colors["CenterLine"])
        plot = self.ax.add_collection(lc)
        self.static_plots["CenterLine"] = [plot]

        no_line_type = False
        unknown_line_type = False
        for l in gt.lane_boundary:
            i=0
            type = l.classification.type
            if type == 4:  # dashed line
                points = []  # create one list for polyline
            else:
                points = [[],[]]  # create lists for x and y data separately, for solid line plots

            if type == 3 or type == 4 or type == 5:  # solid line or dashed line or botts dots
                color = '#222222'
            else:
                if type == 2:
                    color = '#DDDDDD'  # light gray for NO_LINE
                    no_line_type = True
                else:
                    color = '#FF9999'  # light red as default for various unsupported line types
                    unknown_line_type = True

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
                pass
                lc = mc.LineCollection(points, label=self.rmtype2string(type), color=color, picker=5)
                plot = self.ax.add_collection(lc)
            else:  # solid line
                plot, = self.ax.plot(points[0], points[1], label=self.rmtype2string(type), color=color, picker=5)

            self.plot_colors[self.rmtype2string(type)] = color

            # group plots by line type for visibility
            if not self.rmtype2string(type) in self.static_plots:
                self.static_plots[self.rmtype2string(type)] = []
            self.static_plots[self.rmtype2string(type)].append(plot)

        if no_line_type:
            print('Boundary type {} plotted with light gray color'.format(self.rmtype2string(type)))
        if unknown_line_type:
            print('Unsupported lane boundary type: {} plotted with light red color'.format(self.rmtype2string(type)))

        for i, rm in enumerate(gt.road_marking):
            self.x_roadmark = []
            self.y_roadmark = []
            bps = rm.base.base_polygon
            for bp in bps:
                self.x_roadmark.append(bps.x)
                self.y_roadmark.append(bps.y)
            self.ax.plot(self.x_roadmark, self.y_roadmark, color='#333333', label='RoadMarking' if i==0 else '', picker=5)

        # Connect the pick event
        self.fig.canvas.mpl_connect('pick_event', self.on_pick)

    # Function to handle picking events
    def on_pick(self, event):
        # Check if the picked artist is the LineCollection
        if isinstance(event.artist, mc.LineCollection):
            ind = event.ind  # Get the index of the segment that was clicked
            self.text.set_val('LineCollection: {}'.format(event.ind[0]))
        elif isinstance(event.artist, lines.Line2D):
            ind = event.ind  # Get the index of the segment that was clicked
            self.text.set_val('Line: {}'.format(event.ind[0]))

    def toggle_visibility(self, label):
        for plot in self.static_plots[label]:
            plot.set_visible(not plot.get_visible())

    def add_dynamic_content(self):
        # Dynamic content: Plot a dynamic line (that will update)
        self.dynamic_line, = self.ax.plot([], [], color='red', label='Dynamic Line', linewidth=2)
        # Set x and y limits for the dynamic line
        # self.ax.set
        # self.ax.set_xlim(0, 2*np.pi)
        # self.ax.set_ylim(-1.5, 1.5)

        # Show legend
        # self.ax.legend()

    # Function to update the dynamic plot (called repeatedly)
    def update(self, frame):
        self.x_dynamic = np.linspace(0, 2*np.pi, 100)
        self.y_dynamic = np.sin(self.x_dynamic + frame / 10.0)  # Dynamic change
        self.dynamic_line.set_data(self.x_dynamic, self.y_dynamic)
        return self.dynamic_line,

    # Button action to toggle grid visibility
    def toggle_grid(self, event):
        self.ax.grid(not self.ax.xaxis._major_tick_kw['gridOn'])  # Toggle grid visibility

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

        self.view = View(self.gt)


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
        plt.show()

    exit (0)

