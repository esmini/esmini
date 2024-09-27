## Convert pure line geometry roads into parametric cubic polynomials
## achieving tangent continuity, ensuring no geometry gaps or overlaps.
##
## The output is a modified OpenDRIVE file, "_curves" appended to input filename.
## The original file is preserved.
##
## Usage: xodr_lines2curves.py <OpenDRIVE file>
##
## Overall algorithm:
##  - remove duplicate roads, keep only last
##  - look for sequences of connected roads containing only line geometries
##  - stop search at junctions and non-line geometries
##  - resulting sequences of roads is converted into single roads with curve geometries
##  - each geometry start point makes a control for the spline
##  - start and end heading of the spline are constrained to the original geometry
##  - lane sections are interpolated and transferred to the single road, with adjusted s values
##
## Limitations:
##  - road objects are not considered, hence deleted from all but first road in each sequence
##  - only successor roads connected head-to-toe, i.e. equally directed, are considered
##  - short lane section are not merged, which can result in shaky motion in spite of interpolation
##  - no error handling, hopefully output from Python interpreter gives clues on failure


import os
import xml.etree.ElementTree as etree
import sys
import math
import matplotlib.pyplot as plt
import numpy as np
import subprocess
import argparse
from scipy.interpolate import CubicSpline
from scipy.integrate import quad


min_geom_len = 10  # shorter geometries and roads will be merged

parser = argparse.ArgumentParser(prog=os.path.basename(sys.argv[0]), description='Convert line based roads to parametric curves')
parser.add_argument('input_filename', help='required input OpenDRIVE file')
parser.add_argument('output_filename', nargs='?', help='optional output filename')
args = parser.parse_args()

# calculate natural cubic spline polynomials from x,y points
def CreateSpline(x, y, p, heading_start, heading_end):
    xs = CubicSpline(p,x,bc_type=(
        (1, math.cos(heading_start) if heading_start else (x[1]-x[0])/(p[1]-p[0])),
        (1, math.cos(heading_end) if heading_end else (x[-1]-x[-2])/(p[-1]-p[-2]))))
    ys = CubicSpline(p,y,bc_type=(
        (1, math.sin(heading_start) if heading_start else (y[1]-y[0])/(p[1]-p[0])),
        (1, math.sin(heading_end) if heading_end else (y[-1]-y[-2])/(p[-1]-p[-2]))))
    return xs, ys

# used for finding arc length of parametric cubic curve
def integrand(p, bu, cu, du, bv, cv, dv):
    return np.sqrt(
        (3*du*p**2 + 2*cu*p + bu)**2 +
        (3*dv*p**2 + 2*cv*p + bv)**2)

def append_string_to_filename_stem(str, filename):
    stem, ext = os.path.splitext(filename)
    return stem + str + ext

# utility functions for navigating in the OpenDRIVE XML
def find_road(roads, id):
    for j, road_tmp in enumerate(roads):
        if road_tmp.attrib['id'] == id:
            return road_tmp
    return None

def get_first_predecessor(roads, road, max_depth = 10000, visited = []):
    visited.append(road)
    if (len(visited) > max_depth):
        print('Giving up finding first predecessor after', max_depth, 'links. If needed, increase setting in code')
    link = road.find('link')
    if (link is not None):
        if ((predecessor := link.find('predecessor')) is not None):
            rid = road.attrib['id']
            pid = predecessor.attrib['elementId']
            if (predecessor.attrib['elementType'] == 'junction'):
                print('Road {} connected to junction {}, stop search for first predecessor'.format(rid, pid))
                return road  # found junction, stop search
            if (predecessor.attrib['contactPoint'] == 'start'):
                print('Road {} connects to start point of predecessor {}, stop search for first predecessor'.format(rid, pid))
                return road
            if ((road_tmp := find_road(roads, pid)) is not None):
                if not road_tmp in visited:
                    return get_first_predecessor(roads, road_tmp)
    return road

def get_next_successor(roads, road):
    road_tmp = None
    link = road.find('link')
    if (link is not None):
        successor = link.find('successor')
        if (successor is not None):
            road_tmp = find_road(roads, successor.attrib['elementId'])
    return road_tmp

def removeRoadWithDuplicateIds(xodr):  # keep latest
    ids = []
    removed = 0
    i = 0
    while i < len(roads := xodr.findall('road')):
        id = roads[i].attrib['id']
        duplicate = False
        for entry in ids:
            if (entry[0] == id):
                xodr.getroot().remove(entry[1])
                ids.remove(entry)
                print('Removing fist occurrence of duplicate Road', entry[0])
                duplicate = True
                removed += 1
                break
        if (not duplicate):
            ids.append([id, roads[i]])
            i += 1
    return removed

def find_lane_by_id(id, lane_section):
    side_str = 'right' if float(id) < 0 else 'left'
    side = lane_section.find(side_str)
    if (side is not None):
        return side.find('lane[@id="{}"]'.format(id))

def interpolate_lane_width(road):
    # for every lane section and lane, find width of itself and its successor, interpolate in between
    lane_sections = road.find('lanes').findall('laneSection')
    for i, ls in enumerate(lane_sections[:-1]):
        for side_str in ['left', 'right']:
            if ((side := ls.find(side_str)) is not None):
                for j, lane in enumerate(side.findall('lane')):
                    if ((successor := lane.find('link/successor')) is not None):
                        if ((lane2 := find_lane_by_id(successor.attrib['id'], lane_sections[i+1])) is not None):
                            w1 = float(lane.find('width').attrib['a'])
                            w2 = float(lane2.find('width').attrib['a'])
                            length = float(lane_sections[i+1].attrib['s']) - float(ls.attrib['s'])
                            lane.find('width').attrib['b'] = str((w2 - w1) / length)


# Open OpenDRIVE file
odr = args.input_filename
odr_tree = etree.parse(odr)

# delete any duplicate roads, keeping only the last one
removed = removeRoadWithDuplicateIds(odr_tree)

# investigate all roads, identify sequences of at least two connected roads
roads = odr_tree.findall(".//road")
nr_roads = len(roads)
roads_processed = []
road_sequences = []
geom_sequences = []
p_map = []  # map old to new p values

for i, road in enumerate(roads):
    if road in roads_processed:
        continue  # road already processed

    # find first road in sequence
    road_tmp = get_first_predecessor(roads, road)
    road_sequence = []

    # road_tmp represents first road in sequence, now move forward
    while road_tmp is not None:
        roads_processed.append(road_tmp)
        if (len(road_tmp.findall('planView/geometry')) != len(road_tmp.findall('planView/geometry/line'))):
            print('Road', road_tmp.attrib['id'], 'has at least one none-line geometry, skipping')
            road_sequence.clear()
            break
        road_sequence.append(road_tmp)
        road_tmp = get_next_successor(roads, road_tmp)

    if (len(road_sequence) > 0):
        road_sequences.append(road_sequence)

# process all connected road sequences, replace lines with parametric cubic curves
for i, road_sequence in enumerate(road_sequences):
    # create control points for the spline curve representing a sequence of roads
    x = []
    y = []
    p = []
    lane_sections = []
    s_road = 0.0

    for j, road in enumerate(road_sequence):
        for ls in road.findall('lanes/laneSection'):
            lane_sections.append([s_road + float(ls.attrib['s']), ls])

        geoms = road.findall(".//geometry")

        if (j == 0):
            h_start = float(geoms[0].attrib['hdg'])
        if (j == len(road_sequence) - 1):
            h_end = float(geoms[-1].attrib['hdg'])

        for k, geom in enumerate(geoms):
            x.append(float(geom.attrib['x']))
            y.append(float(geom.attrib['y']))
            p.append(0.0 if j==0 and k==0 else p[-1] + float(geoms[k-1].attrib['length']))

        # add last point only if last road
        if (j == len(road_sequence) - 1):
            x.append(x[-1] + float(geoms[-1].attrib['length']) * math.cos(h_end))
            y.append(y[-1] + float(geoms[-1].attrib['length']) * math.sin(h_end))
            p.append(p[-1] + float(geoms[-1].attrib['length']))

        s_road += float(road.attrib['length'])

        if (j > 0):
            if (j == len(road_sequence) - 1):
                # transfer any successor connection of the last road to new single road
                if ((link := road_sequence[0].find('link')) is None):
                    link = etree.SubElement(road_sequence[0], 'link')
                if ((successor_old := link.find('successor')) is not None):
                    link.remove(successor_old)
                if ((successor := road_sequence[j].find('link/successor')) is not None):
                    link.append(successor)

            # remove all but the first road element
            odr_tree.getroot().remove(road)

    # remove dense points
    j = 1
    while j < len(x):
        if (math.sqrt(pow(x[j] - x[j-1], 2) + pow(y[j] - y[j-1], 2)) < min_geom_len):
            del(x[j])
            del(y[j])
            del(p[j])
        else:
            j += 1

    # create the spline curve
    xs_plot, ys_plot = CreateSpline(x, y, p, h_start, h_end)

    # calculate the spline arc length and update parameter to represent arc length of the curve instead of the lines
    step_len = 0.1
    p_array = np.arange(p[0], p[-1] + step_len/10.0, step_len)
    new_p = [0.0]
    p_map.append([0.0, 0.0])
    for i in range(len(p)-1):
        length = quad(integrand,0,p[i+1]-p[i], (
            xs_plot.c[2, i], xs_plot.c[1, i], xs_plot.c[0, i],
            ys_plot.c[2, i], ys_plot.c[1, i], ys_plot.c[0, i]))[0]
        new_p.append(new_p[i] + length)
        p_map.append([p[i+1], new_p[-1]])

    # recreate the spline based on the new parameterization
    xs_odr, ys_odr = CreateSpline(x, y, new_p, h_start, h_end)

    # replace all geometries in the original road
    pivot_road = road_sequence[0]
    plan_view = pivot_road.find('planView')
    for geom in plan_view.findall('geometry'):
        plan_view.remove(geom)

    geom_counter = 0
    for j in range(len(p)-1):
        # create the cubic parametric curve - separate cubic polynomials for x and y
        geom = etree.SubElement(plan_view, 'geometry')
        h = np.arctan2(ys_odr.c[2,geom_counter], xs_odr.c[2,geom_counter])
        etree.SubElement(geom, 'paramPoly3',
                            pRange="arcLength",
                            aU = "0.0",
                            bU = str(xs_odr.c[2, geom_counter] * np.cos(-h) - ys_odr.c[2, geom_counter] * np.sin(-h)),
                            cU = str(xs_odr.c[1, geom_counter] * np.cos(-h) - ys_odr.c[1, geom_counter] * np.sin(-h)),
                            dU = str(xs_odr.c[0, geom_counter] * np.cos(-h) - ys_odr.c[0, geom_counter] * np.sin(-h)),
                            aV = "0.0",
                            bV = "0.0",
                            cV = str(xs_odr.c[1, geom_counter] * np.sin(-h) + ys_odr.c[1, geom_counter] * np.cos(-h)),
                            dV = str(xs_odr.c[0, geom_counter] * np.sin(-h) + ys_odr.c[0, geom_counter] * np.cos(-h))),

        # update some attributes of the geometry element
        geom.attrib['x'] = str(x[j])
        geom.attrib['y'] = str(y[j])
        geom.attrib['s'] = str(new_p[j])
        geom.attrib['hdg'] = str(h)
        geom.attrib['length'] = str(new_p[geom_counter+1] - new_p[geom_counter])
        geom_counter += 1

    pivot_road.remove(pivot_road.find('lanes'))
    lanes = etree.SubElement(pivot_road, 'lanes')
    for ls_entry in lane_sections:
        # find new s-value by interpolation interpolation
        for j, entry in enumerate(p_map[:-1]):
            if ls_entry[0] > entry[0] - 1e-6:
                s = p_map[j][1] + (ls_entry[0] - p_map[j][0]) / (p_map[j+1][0] - p_map[j][0]) * (p_map[j+1][1] - p_map[j][1])
                ls_entry[1].attrib['s'] = str(s)
                break
        lanes.append(ls_entry[1])

    interpolate_lane_width(pivot_road)

    pivot_road.attrib['length'] = str(new_p[-1])

    # plot both lines and curve for comparison
    plt.gca().set_aspect('equal')
    plt.plot(x, y)
    plt.plot(xs_plot(p_array), ys_plot(p_array))

if args.output_filename is not None:
    odr_out_filename = args.output_filename
else:
    # add "_curves" to the original filename
    odr_out_filename = append_string_to_filename_stem("_curves", odr)

# write the manipulated OpenSCENARIO file
odr_tree.write(odr_out_filename)
nr_roads_considered = 0
for seq in road_sequences:
    nr_roads_considered += len(seq)

if (removed > 0):
    print('Removed', removed, "duplicate roads")
if (nr_roads_considered < nr_roads):
    print('Ignored {} road{} containing at least one non-line geometry'.format(nr_roads - nr_roads_considered, "" if (nr_roads - nr_roads_considered == 1) else "s"))

print('Processed {} of total {} roads. Found {} sequences of pure successor linked roads'.format(nr_roads_considered, nr_roads, len(road_sequences)))
print('Updated OpenDRIVE written to {0}'.format(odr_out_filename))

plt.show()
subprocess.run(["/tmp/esmini/bin/odrviewer", "--density", "5", "--window", "60", "60", "800", "400", "--odr", odr_out_filename])
