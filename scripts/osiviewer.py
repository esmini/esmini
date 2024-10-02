# esmini - Environment Simulator Minimalistic
# https://github.com/esmini/esmini
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
#
# Copyright (c) partners of Simulation Scenarios
# https://sites.google.com/view/simulationscenarios


import matplotlib.pyplot as plt
import matplotlib.animation as anim
import matplotlib.collections as mc
import matplotlib.widgets as mw
import matplotlib.patches as patches
import matplotlib.transforms as transforms
import numpy as np
import os
import sys
import struct
import time

# Add scripts root directory to module search path in order to find osi3
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../', 'scripts')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'scripts')))  # if script moved to Hello World folder

from osi3.osi_groundtruth_pb2 import GroundTruth


class BBObject:
    def __init__(self, id, x, y, h, v, width, height, axes):
        self.id = id
        self.width = width
        self.height = height
        self.kf = []
        self.patch = None
        self.ref_point = None
        self.ax = axes
        self.current_idx = 0
        self.draw()
        self.update(x, y, h, v)

    def add_keyframe(self, timestamp, x, y, h, v):
        self.kf.append((timestamp, x, y, h, v))

    def draw(self):
        # reference point at center of the bounding box
        self.patch = plt.Rectangle((-self.width/2.0, -self.height/2.0), self.width, self.height, label='object_{}'.format(self.id), fill=False, edgecolor='red', lw=1, picker=5, zorder=5)
        self.ax.add_patch(self.patch)
        self.transform = transforms.Affine2D()
        self.patch.set_transform(self.transform + self.ax.transData)
        self.ref_point = plt.Circle((0.0, 0.0), 0.3, fill=True, edgecolor='red', lw=1)
        self.ax.add_patch(self.ref_point)
        self.ref_point.set_transform(self.transform + self.ax.transData)

    def update(self, x, y, h, v):
        self.x = x
        self.y = y
        self.h = h
        self.v = v
        self.transform.clear().rotate_deg(np.rad2deg(self.h)).translate(self.x, self.y)

    def set_time(self, time, interpolate=True):
        i=0
        if self.kf is None or len(self.kf) == 0:
            return
        if time >= self.kf[self.current_idx][0]:
            # look forward from current frame
            for i in range(self.current_idx, len(self.kf)-1):
                if time < self.kf[i+1][0]:
                    break
        else:
            # look backwards
            for i in range(self.current_idx,-1,-1):
                if time >= self.kf[i][0]:
                    break

        if interpolate:
            i2 = min(i+1, len(self.kf)-1)
            dt = self.kf[i2][0] - self.kf[i][0]
            if dt > 1e-6:
                w = (time - self.kf[i][0]) / dt
            else:
                w = 0.0  # no time diff, go for first keyframe
            v = [v for v in self.kf[i]]
            # simple interpolation for x, y, v
            for j in [1, 2, 4]:
                v[j] += w*(self.kf[i2][j] - self.kf[i][j])
            # angle need special handling
            angels = np.unwrap([self.kf[i][3], self.kf[i2][3]])
            v[3] = np.mod(angels[0] + w*(angels[1] - angels[0]), 2 * np.pi)
        else:
            v = self.kf[i]
        self.update(*v[1:])
        self.current_idx = i

class BBObjects:
    def __init__(self):
        self.bb_objects = []

    def add_bb_object(self, bb_object):
        self.bb_objects.append(bb_object)

    def get_bb_object(self, id):
        for bb in self.bb_objects:
            if bb.id == id:
                return bb
        return None

    def set_time(self, current_time, interpolate=True):
        for bb in self.bb_objects:
            bb.set_time(current_time, interpolate)

class View:
    def __init__(self, gt):

        # settings
        self.empty_select_string = ''
        self.grid_enabled = True
        self.font_size = 9

        # initialize variables
        self.gt = gt
        self.highlight = None
        self.selected_object = None
        self.playing = True
        self.pan_start = None
        self.y_start = None
        self.follow_object_index = -1
        self.mod = {'shift': False}
        self.picked = False
        self.bb_objects = BBObjects()
        self.nr_objects = len(self.gt.moving_object)

        # create the figure canvas and axis
        px = 1/plt.rcParams['figure.dpi']  # pixel size in inches
        plt.rcParams['toolbar'] = 'none'
        self.fig, self.ax = plt.subplots(figsize=(1200*px, 600*px))
        self.ax.grid(self.grid_enabled)
        self.ax.set_aspect('equal', 'datalim')
        self.ax.tick_params(axis='x', labelsize=self.font_size)
        self.ax.tick_params(axis='y', labelsize=self.font_size)

        # add static content from ground-truth, currently limited to the road network
        self.add_static_content(self.gt)  # static content assumed to be in first gt message only

        # plot the static content
        self.ax.plot()
        self.xlim = self.ax.get_xlim()
        self.ylim = self.ax.get_ylim()

        self.create_gui()

        # add idle function for updating dynamic content
        self.ani = anim.FuncAnimation(self.fig, self.update, frames=range(100), blit=False, interval=50)

    def redraw(self):
        plt.draw()

    def create_gui(self):
        # CheckButtons widget for visibility
        labels = list(self.static_plots_by_type.keys())
        self.cbax = plt.axes([1.0, 0.5, 2.0, 0.04*len(labels)], frameon=False)  # Position of the checkbox area

        colors = []
        fontsizes = [self.font_size]*len(labels)
        for l in labels:
            colors.append(self.plot_colors[l])
        self.check = mw.CheckButtons(self.cbax, labels, [True]*len(labels), label_props={'color': colors, 'fontsize': fontsizes})
        self.check.on_clicked(self.toggle_visibility)

        # grid toggle button
        self.gbax = plt.axes([0.05, 0.05, 0.2, 0.075])
        self.button_grid = mw.Button(self.gbax, "toggle grid ('g')")
        self.button_grid.label.set_fontsize(self.font_size)
        self.button_grid.on_clicked(self.toggle_grid)

        # play/pause button
        self.pcax = plt.axes([1.0, 0.5, 2.0, 0.04])
        self.button_play = mw.Button(self.pcax, "play / pause ('space')")
        self.button_play.label.set_fontsize(self.font_size)
        self.button_play.on_clicked(self.toggle_play)

        # repeat checkbox
        self.checks_ax = plt.axes([0.0, 0.0, 0.9, 0.03], frameon=False)
        self.checks = mw.CheckButtons(self.checks_ax, ["repeat", "interpolate"], [True]*2, label_props={'color':['#000000']*2, 'fontsize': [self.font_size]*2})

        # zoom extents button
        self.zeax = plt.axes([1.0, 0.5, 2.0, 0.04])
        self.button_zoom_extents = mw.Button(self.zeax, "zoom extents ('z')")
        self.button_zoom_extents.label.set_fontsize(self.font_size)
        self.button_zoom_extents.on_clicked(self.zoom_extents)

        # text area
        self.tax = plt.axes([1.0, 0.5, 2.0, 20])
        self.tax.set_axis_off()
        self.text = self.tax.text(0.0, 0.0, self.empty_select_string, fontsize=self.font_size)
        self.text.set_fontname('monospace')

        # help text area
        self.hax = plt.axes([1.0, 0.5, 2.0, 20])
        self.hax.set_axis_off()
        self.htext = self.hax.text(0.0, 0.0, self.empty_select_string, fontsize=self.font_size-0.5, va='bottom')
        htext = "Help:\n"
        htext += "play/pause: space\ncycle object: tab/shift-tab\nrelease object: ctrl+tab\nselect: left mouse button (mb)\n"
        htext += "zoom: scroll or right mb or ,/.\nzoom extents: z\npan: middle mouse button\n"
        htext += "step: left/right\n1 sec back/fwd: shift+left/right\n"
        htext += "to start/end: ctrl+left/right\ngrid on/off: g\nfullscreen toggle: f\nquit: q\n"
        self.htext.set_text(htext)

        # time slider
        self.tsax = plt.axes([0.0, 0.0, 0.9, 0.03])
        self.time_slider = mw.Slider(
            ax=self.tsax,
            label='Time [s]',
            valmin=0.0,
            valmax=None,
            valinit=0.0,
            valfmt='%.2f'
        )
        self.time_slider.label.set_horizontalalignment('left')
        self.time_slider.label.set_fontsize(self.font_size)
        self.time_slider.valtext.set_font('monospace')
        self.time_slider.valtext.set_fontsize(1.6*self.font_size)
        self.time_slider.valtext.set_horizontalalignment('right')
        self.time_slider.on_changed(self.set_time)

        self.last_system_time = time.time()
        self.current_time = 0.0
        self.last_timestamp = 0.0

        # Connect the pick event
        self.fig.canvas.mpl_connect('pick_event', self.on_pick)
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        self.fig.canvas.mpl_connect('key_release_event', self.on_key_release)
        self.fig.canvas.mpl_connect('resize_event', self.on_resize)
        self.fig.canvas.mpl_connect('scroll_event', self.on_scroll)
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_motion)

        self.fig.canvas.manager.set_window_title('OSI viewer')

    def add_bb_object(self, bb_object):
        self.bb_objects.add_bb_object(bb_object)
        if self.follow_object_index == -1:
            self.select_object(index=0)
            self.zoom_radius(bb_object.x, bb_object.y, radius=25.0)


    def pan(self, x, y):
        if self.pan_start is not None:
            dx = x - self.pan_start[0]
            dy = y - self.pan_start[1]
            xlim = self.ax.get_xlim()
            ylim = self.ax.get_ylim()
            self.ax.set_xlim((xlim[0] - dx, xlim[1] - dx))
            self.ax.set_ylim((ylim[0] - dy, ylim[1] - dy))
            self.redraw()

    def zoom_radius(self, x, y, radius):
        self.ax.set_xlim((x - radius, x + radius))
        self.ax.set_ylim((y - radius, y + radius))
        self.redraw()

    def zoom_factor(self, factor, center):
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        dx = (xlim[1] - xlim[0])/2
        dy = (ylim[1] - ylim[0])/2
        x = center[0] + (xlim[0] + dx - center[0]) * factor
        y = center[1] + (ylim[0] + dy - center[1]) * factor
        dx *= factor
        dy *= factor
        self.ax.set_xlim((x - dx, x + dx))
        self.ax.set_ylim((y - dy, y + dy))
        self.redraw()

    def zoom_extents(self, event=None):
        self.unselect_object()
        self.ax.set_xlim(self.xlim)
        self.ax.set_ylim(self.ylim)
        self.follow_object_index = -1
        self.redraw()

    def on_motion(self, event):
        if event.xdata is None or event.ydata is None:
            return
        if event.button == 2:
            self.pan(event.xdata, event.ydata)
        elif event.button == 3:
            if self.selected_object is not None:
                center = (self.bb_objects.bb_objects[self.follow_object_index].x, self.bb_objects.bb_objects[self.follow_object_index].y)
            else:
                center = (self.pan_start[0], self.pan_start[1])
            self.zoom_factor(1.0 - 0.0025*(self.y_start - event.y), center=center)
            self.y_start = event.y

    def on_scroll(self, event):
        if self.selected_object is not None:
            center = (self.bb_objects.bb_objects[self.follow_object_index].x, self.bb_objects.bb_objects[self.follow_object_index].y)
        else:
            center = (event.xdata, event.ydata)
        self.zoom_factor(factor=1.0 - (event.step/10.0), center=center)

    def on_key_press(self, event):
        if event.key == 'shift':
            self.mod['shift'] = True
        elif event.key == ' ':
            self.toggle_play(event)
        elif event.key == 'left':
            self.step(-1)
        elif event.key == 'ctrl+left':
            self.set_slider_time(0.0, pause=True)
        elif event.key == 'shift+left':
            self.set_slider_time(max(self.current_time - 1.0, 0), pause=True)
        elif event.key == 'right':
            self.step(1)
        elif event.key == 'ctrl+right':
            self.set_slider_time(self.last_timestamp, pause=True)
        elif event.key == 'shift+right':
            self.set_slider_time(min(self.current_time + 1.0, self.last_timestamp), pause=True)
        elif event.key == 'tab' or event.key == 'ctrl+tab':
            self.nr_objects = len(self.gt.moving_object)
            if self.nr_objects > 0:
                if event.key == 'ctrl+tab':  # release selection and camera
                    self.follow_object_index = -1
                else:
                    if self.mod['shift']:  # select previous
                        self.follow_object_index -= 1
                    else:  # select next
                        self.follow_object_index += 1
                    # wrap around
                    if self.follow_object_index >= self.nr_objects:
                        self.follow_object_index = 0
                    elif self.follow_object_index < 0:
                        self.follow_object_index = self.nr_objects - 1

                self.select_object(index=self.follow_object_index)
                self.update_view()
        elif event.key == ',' or event.key == '.':
            if self.selected_object is not None:
                center = (self.bb_objects.bb_objects[self.follow_object_index].x, self.bb_objects.bb_objects[self.follow_object_index].y)
            else:
                center = (event.xdata, event.ydata)
            self.zoom_factor(0.5 if event.key == ',' else 1.5, center=center)
        elif event.key == 'z':
            self.zoom_extents()

    def on_key_release(self, event):
        if event.key == 'shift':
            self.mod['shift'] = False

    def on_click(self, event):
        if event.xdata is None or event.ydata is None:
            return
        if event.button == 2 or event.button == 3:
            self.pan_start = (event.xdata, event.ydata)
            if event.button == 3 :
                self.y_start = event.y

    def on_release(self, event):
        if event.button == 2:
            self.pan_start = None
        elif event.button == 1 and event.inaxes == self.ax and not self.picked:
            self.unselect()
            self.unselect_object()
            self.redraw()
        self.picked = False

    def on_resize(self, event):
        self.adjust_margins()

    def on_pick(self, event):
        if event.mouseevent.button != 1:
            return  # only interested in left button clicks for picking
        self.unselect()
        self.unselect_object()
        self.picked = True
        if isinstance(event.artist, patches.Rectangle) or isinstance(event.artist, patches.Polygon):
            self.select_object(artist=event.artist)
        elif isinstance(event.artist, mc.LineCollection) or isinstance(event.artist, mc.PathCollection):
            index = self.osi_idx_by_collection[event.artist][event.ind[0]]
            id = self.osi_ids_by_collection[event.artist][event.ind[0]]
            if isinstance(event.artist, mc.LineCollection):
                lc = mc.LineCollection([event.artist.get_segments()[event.ind[0]]], colors=event.artist.get_colors(), linewidths=3.0)
                self.highlight = self.ax.add_collection(lc)
                self.update_pick_text(event.artist.get_label(), index, id, event.ind[0])
            elif isinstance(event.artist, mc.PathCollection):
                self.update_pick_text(event.artist.get_label(), self.osi_idx_by_collection[event.artist][0], self.osi_ids_by_collection[event.artist][0], event.ind[0])
                offsets = event.artist.get_offsets()
                self.highlight = self.ax.scatter(offsets[event.ind[0]][0], offsets[event.ind[0]][1], label=self.rmtype2string(type), s=18.0, color=event.artist.get_facecolors()[0])
        else:
            self.text.set_val('Unknown')
        self.redraw()

    def set_last_timestamp(self, timestamp):
        self.last_timestamp = timestamp
        if self.last_timestamp == 0:
            self.time_slider.ax.set_xlim(0, None)
        else:
            self.time_slider.ax.set_xlim(0, self.last_timestamp)
            self.time_slider.valmax = self.last_timestamp

    def set_slider_time(self, val, pause=True):
        if pause:
            self.pause()
        self.time_slider.set_val(val)

    def set_time(self, val):
        self.current_time = val
        self.bb_objects.set_time(self.current_time, self.checks.get_status()[1])
        self.update_follow_text()
        self.update_view()

    def step(self, val):
        if len(self.bb_objects.bb_objects) == 0:
            return
        # pick selected or first object as reference
        obj_index = max(self.follow_object_index, 0)
        obj = self.bb_objects.bb_objects[obj_index]
        # find the requested keyframe
        frame = obj.kf[max(min(obj.current_idx+val, len(obj.kf)-1), 0)]
        self.set_slider_time(frame[0], pause=True)

    def update_view(self):
        if self.follow_object_index >= 0:
            xlim = self.ax.get_xlim()
            ylim = self.ax.get_ylim()
            obj = self.bb_objects.bb_objects[self.follow_object_index]
            self.ax.set_xlim(obj.x - (xlim[1] - xlim[0])/2, obj.x + (xlim[1] - xlim[0])/2)
            self.ax.set_ylim(obj.y - (ylim[1] - ylim[0])/2, obj.y + (ylim[1] - ylim[0])/2)
        if not self.fig.stale:
            self.redraw()

    def pause(self):
        if self.playing:
            self.playing = False
            self.ani.event_source.stop()

    def play(self):
        if not self.playing:
            self.playing = True
            if self.current_time >= self.last_timestamp:
                self.current_time = 0.0
            self.last_system_time = time.time()
            self.ani.event_source.start()
            self.redraw()

    def toggle_play(self, event):
        if self.playing:
            self.pause()
        else:
            self.play()
        self.redraw()

    # Adjust the axis margins (padding between the axis and the figure borders)
    def adjust_margins(self):
        menu_width = 200
        margin = 10
        self.screen_width, self.screen_height = self.fig.canvas.get_width_height()

        y = 10
        self.fig.subplots_adjust(left=70.0/self.screen_width,
                                 right=1.0-(2*margin + menu_width)/self.screen_width,
                                 top=1.0-y/self.screen_height,
                                 bottom=75/self.screen_height,)
        y = 2
        cb_height = 20*len(self.static_plots_by_type.keys())
        y += cb_height
        self.cbax.set_position([1.0-(25 + menu_width)/self.screen_width,
                               1-y/self.screen_height,
                               menu_width/self.screen_width,
                               cb_height/self.screen_height])
        y += 0 + 25
        self.pcax.set_position([1.0-(margin + menu_width)/self.screen_width,
                               1-y/self.screen_height,
                               menu_width/self.screen_width,
                               25/self.screen_height])
        y += 0 + 45
        self.checks_ax.set_position([1.0-(25 + menu_width)/self.screen_width,
                               1-y/self.screen_height,
                               menu_width/self.screen_width,
                               45/self.screen_height])
        y += 0 + 25
        self.gbax.set_position([1.0-(margin + menu_width)/self.screen_width,
                               1-y/self.screen_height,
                               menu_width/self.screen_width,
                               25/self.screen_height])
        y += margin + 25
        self.zeax.set_position([1.0-(margin + menu_width)/self.screen_width,
                               1-y/self.screen_height,
                               menu_width/self.screen_width,
                               25/self.screen_height])
        y += margin + 60
        self.tax.set_position([1.0-(margin + menu_width)/self.screen_width,
                               1-y/self.screen_height,
                               menu_width/self.screen_width,
                               60/self.screen_height])
        y += margin + 228
        self.hax.set_position([1.0-(margin + menu_width)/self.screen_width,
                               1-y/self.screen_height,
                               menu_width/self.screen_width,
                               228/self.screen_height])

        y = self.screen_height - 25
        self.tsax.set_position([10/self.screen_width, 10/self.screen_height, 1.0-(20/self.screen_width), 25/self.screen_height])
        self.time_slider.label.set_position((0/self.screen_width, 1.2))
        self.time_slider.valtext.set_position((1.0 - 5/self.screen_width, 1.3))

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

    def unselect_object(self):
        if self.selected_object:
            self.selected_object.set_linewidth(1.0)
            self.selected_object = None
            self.follow_object_index = -1
        self.text.set_text(self.empty_select_string)

    def unselect(self):
        if self.highlight:
            self.highlight.remove()
            self.highlight = None
        self.text.set_text(self.empty_select_string)

    def add_static_content(self, gt):
        self.static_plots_by_type = {}
        self.osi_ids_by_collection = {}
        self.osi_idx_by_collection = {}
        self.osi_ids_by_stationary = {}
        self.osi_idx_by_stationary = {}
        self.plot_colors = {}
        lines = []
        ids = []
        idx = []
        for index, lane in enumerate(gt.lane):
            clf=lane.classification
            if len(clf.centerline) > 0:
                i=0
                line=[]
                while i < len(clf.centerline):
                    p0 = clf.centerline[i]
                    line += [(p0.x, p0.y)]
                    i += 1
                ids.append(lane.id.value)
                idx.append(index)
                lines.append(line)
            else:
                pass  # probably in junction, no centerline, only lane pairings

        self.plot_colors["CenterLine"] = '#BBBBFF'
        collection = mc.LineCollection(lines, picker=5, label="CenterLine", color=self.plot_colors["CenterLine"])
        self.osi_ids_by_collection[collection] = ids
        self.osi_idx_by_collection[collection] = idx
        plot = self.ax.add_collection(collection)
        self.static_plots_by_type["CenterLine"] = [plot]

        no_line_type = False
        unknown_line_type = False
        for index, l in enumerate(gt.lane_boundary):
            ids = []
            indices = []
            i=0
            type = l.classification.type
            points = []  # create one list for polyline
            scatter_points = [[],[]]
            indices.append(index)

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
                p = l.boundary_line[i].position
                if type == 5:  # botts dots
                    scatter_points[0].append(p.x)
                    scatter_points[1].append(p.y)
                if type == 4:  # dashed line
                    # one polyline per collection dash segment
                    if i < len(l.boundary_line)-1:
                        p2 = l.boundary_line[i+1].position
                    else:
                        p2 = p  # make a dot instead of line
                    points.append([(p.x, p.y), (p2.x, p2.y)])
                    ids.append(l.id.value)
                    indices.append(index)
                    i += 2
                else:
                    if i==0:
                        # only one polyline per collection
                        ids.append(l.id.value)
                        indices.append(index)
                        points.append([])
                    points[0]+=([(p.x, p.y)])
                    i += 1

            if type == 5:  # botts dots
                collection = self.ax.scatter(scatter_points[0], scatter_points[1], label=self.rmtype2string(type), color=color, s=3.0, picker=5)
                plot = self.ax.add_collection(collection)
            else:
                collection = mc.LineCollection(points, label=self.rmtype2string(type), color=color, picker=5)
                plot = self.ax.add_collection(collection)

            ids.append(l.id.value)
            indices.append(index)
            self.plot_colors[self.rmtype2string(type)] = color

            # group plots by line type for visibility
            if not self.rmtype2string(type) in self.static_plots_by_type:
                self.static_plots_by_type[self.rmtype2string(type)] = []
            self.static_plots_by_type[self.rmtype2string(type)].append(plot)

            self.osi_ids_by_collection[collection] = ids
            self.osi_idx_by_collection[collection] = indices

        if no_line_type:
            print('Boundary type {} plotted with light gray color'.format(self.rmtype2string(type)))
        if unknown_line_type:
            print('Unsupported lane boundary type: {} plotted with light red color'.format(self.rmtype2string(type)))

        # road markings
        for i, rm in enumerate(gt.road_marking):
            bps = rm.base.base_polygon
            # self.x_roadmark = []
            # self.y_roadmark = []
            # for bp in bps:
            #     self.x_roadmark.append(bp.x)
            #     self.y_roadmark.append(bp.y)
            # self.ax.plot(self.x_roadmark, self.y_roadmark, color='#000088', label='RoadMarking' if i==0 else '', picker=5)
            vertices = []
            for bp in bps:
                vertices.append((bp.x, bp.y))
            patch = self.ax.add_patch(patches.Polygon(vertices, label="road_marking".format(rm.id.value, index), facecolor='#FFFFFF', edgecolor='black', linewidth=1, picker=5))
            self.osi_ids_by_stationary[patch] = rm.id.value
            self.osi_idx_by_stationary[patch] = i
            print('vertices:', vertices)

        # stationary objects
        for index, s in enumerate(gt.stationary_object):
            if s.HasField('base'):
                b = s.base
                hdg = b.orientation.yaw
                patch = None
                if len(b.base_polygon) > 0:
                    vertices = []
                    for p in b.base_polygon:
                        vertices.append((p.x, p.y))
                    patch = self.ax.add_patch(patches.Polygon(vertices, label="stationary_polygon".format(s.id.value, index), facecolor='#CCCCCC', edgecolor='black', linewidth=1, picker=5, fill=True, zorder=2))
                    patch.set_transform(transforms.Affine2D().rotate_deg(np.rad2deg(0)).translate(b.position.x, b.position.y) + self.ax.transData)
                else:
                    w = b.dimension.width
                    l = b.dimension.length
                    patch = self.ax.add_patch(plt.Rectangle((-l/2, -w/2), l, w, label="stationary_bb".format(s.id.value, index), facecolor='#CCCCCC', edgecolor='black', lw=1, picker=5, fill=True, zorder=2))
                    patch.set_transform(transforms.Affine2D().rotate_deg(np.rad2deg(hdg)).translate(b.position.x, b.position.y) + self.ax.transData)
                self.osi_ids_by_stationary[patch] = s.id.value
                self.osi_idx_by_stationary[patch] = index

    def update_pick_text(self, label, index, id, instance):
        self.text.set_text('selected:\n{}\nidx {} id {} # {}\n'.format(label, index, id, instance))

    def update_follow_text(self):
        if self.follow_object_index >= 0:
            obj = self.bb_objects.bb_objects[self.follow_object_index]
            self.text.set_text('follow: {}\nidx {} id {}\nx {:.2f} y {:.2f}\nh {:.2f} v {:.2f}'.format(obj.patch.get_label(), self.follow_object_index, obj.id, obj.x, obj.y, obj.h, obj.v))

    def select_object(self, artist=None, index=None):
        self.unselect_object()
        if index is not None:
            self.follow_object_index = index
            if index >= 0 and index < len(self.bb_objects.bb_objects):
                artist = self.bb_objects.bb_objects[index].patch
        if artist is not None:
            self.selected_object = artist
            self.selected_object.set_linewidth(1.75)
            if index is None:
                # stationary object?
                if artist in self.osi_ids_by_stationary:
                    self.text.set_text("selected:\n{}\nids {} idx {}\n".format(artist.get_label(), self.osi_ids_by_stationary[artist], self.osi_idx_by_stationary[artist]))
                    return  # no need of update function for stationary objects
                # else moving object?
                else:
                    for i, obj in enumerate(self.bb_objects.bb_objects):
                        if obj.patch == artist:
                            self.follow_object_index = i
        self.update_follow_text()
        self.redraw()

    def toggle_visibility(self, label):
        for plot in self.static_plots_by_type[label]:
            plot.set_visible(not plot.get_visible())
        self.redraw()

    # Function to update the dynamic plot (called repeatedly)
    def update(self, frame):
        dt = time.time() - self.last_system_time
        self.current_time += dt
        if self.current_time > self.last_timestamp:
            if self.checks.get_status()[0] == True:
                self.current_time = 0.0
            else:
                self.current_time = self.last_timestamp
                self.pause()
        self.set_slider_time(self.current_time, pause=False)
        self.last_system_time = time.time()

    # Button action to toggle grid visibility
    def toggle_grid(self, event):
        self.ax.grid(not self.ax.xaxis._major_tick_kw['gridOn'])  # Toggle grid visibility
        self.redraw()

class OSIFile:
    def __init__(self, osi_filename):
        self.filename = osi_filename
        self.gt = GroundTruth()
        self.view = None
        timestamp = 0.0

        try:
            self.file = open(self.filename, 'rb')
        except OSError:
            print('ERROR: Could not open file {} for reading'.format(self.filename))
            exit(-1)

        while self.read_next_message():
            if self.view is None:
                # create viewer, which will extract the static data from ground-truth
                self.view = View(self.gt)
            if len(self.gt.moving_object) > 0:
                # retrieve timestamp
                timestamp = self.gt.timestamp.seconds + self.gt.timestamp.nanos * 1e-9
                for obj in self.gt.moving_object:
                    if obj.HasField('base'):
                        bb = self.view.bb_objects.get_bb_object(obj.id.value)
                        v = np.sqrt(obj.base.velocity.x**2 + obj.base.velocity.y**2)
                        if bb is None:
                            bb = BBObject(obj.id.value, obj.base.position.x, obj.base.position.y, obj.base.orientation.yaw, v, obj.base.dimension.length, obj.base.dimension.width, self.view.ax)
                            self.view.add_bb_object(bb)
                        # add keyframe
                        bb.add_keyframe(timestamp, obj.base.position.x, obj.base.position.y, obj.base.orientation.yaw, v)
        if self.view is not None:
            self.view.set_last_timestamp(timestamp)

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

if __name__ == '__main__':

    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        print('Usage: {} <osi file>'.format(os.path.basename(sys.argv[0])))
        exit(-1)

    osi_file = OSIFile(sys.argv[1])
    plt.show()
    exit (0)
