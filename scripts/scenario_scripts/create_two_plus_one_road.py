"""
  Create a 2 + 1 road (https://en.wikipedia.org/wiki/2%2B1_road)
  Three sections:
    1. Two left lanes + one right lane
    2. One left lane + two right lanes
    3. Two left lanes + one right lane
  Input parameters:
    1. lane width
    2. transition length
  Dependencies:
    pip install scenariogeneration
    esmini (for preview)
"""

from scenariogeneration import xodr
import os
import sys
import argparse


# Parse arguments
parser = argparse.ArgumentParser(description='Create 2+1 road')
parser.add_argument('--lane_width', '-l', default='3.5', help='lane width')
parser.add_argument('--transition_length', '-t', default='50.0', help='length of transition section')
args = parser.parse_args()
lane_width =float(args.lane_width)
transition_length =float(args.transition_length)

print("lane_width:", lane_width)
print("transition_length:", transition_length)
c = -3 * lane_width / transition_length**2
d = 2 * lane_width / transition_length**3

# create the planview and the geometry
planview = xodr.PlanView()
planview.add_geometry(xodr.Line(500))

# create two different roadmarkings
rm_solid = xodr.RoadMark(xodr.RoadMarkType.solid, 0.12)
rm_dashed = xodr.RoadMark(xodr.RoadMarkType.broken, 0.12)

# create a centerlane (same centerlane can be used since no linking is needed for this)
centerlane = xodr.Lane(a=lane_width)
centerlane.add_roadmark(rm_solid)

# create the first lanesection with two left lanes and one right lane
lanesec1 = xodr.LaneSection(0, centerlane)
lane1 = xodr.Lane(a=lane_width)
lane1.add_roadmark(rm_dashed)
lane2 = xodr.Lane(a=lane_width)
lane2.add_roadmark(rm_solid)
lane3 = xodr.Lane(a=lane_width)
lane3.add_roadmark(rm_solid)
lanesec1.add_left_lane(lane1)
lanesec1.add_left_lane(lane2)
lanesec1.add_right_lane(lane3)

# create the second lanesection shifting middle lane from left to right
lanesec2 = xodr.LaneSection(150.0 - 0.5 * transition_length, centerlane)
lane4 = xodr.Lane(a=lane_width, c=c, d=d)
lane5 = xodr.Lane(a=lane_width)
lane5.add_roadmark(rm_solid)
lane6 = xodr.Lane(a=0, c=-c, d=-d)
lane7 = xodr.Lane(a=lane_width)
lane7.add_roadmark(rm_solid)
lanesec2.add_left_lane(lane4)
lanesec2.add_left_lane(lane5)
lanesec2.add_right_lane(lane6)
lanesec2.add_right_lane(lane7)

# create the third lanesection with one left lane and two right lanes
lanesec3 = xodr.LaneSection(150.0 + 0.5 * transition_length, centerlane)
lane8 = xodr.Lane(a=lane_width)
lane8.add_roadmark(rm_solid)
lane9 = xodr.Lane(a=lane_width)
lane9.add_roadmark(rm_dashed)
lane10 = xodr.Lane(a=lane_width)
lane10.add_roadmark(rm_solid)
lanesec3.add_left_lane(lane8)
lanesec3.add_right_lane(lane9)
lanesec3.add_right_lane(lane10)

# create the fourth lanesection shifting middle lane from right to left
lanesec4 = xodr.LaneSection(350.0 - 0.5 * transition_length, centerlane)
lane11 = xodr.Lane(a=0, c=-c, d=-d)
lane12 = xodr.Lane(a=lane_width)
lane12.add_roadmark(rm_solid)
lane13 = xodr.Lane(a=lane_width, c=c, d=d)
lane14 = xodr.Lane(a=lane_width)
lane14.add_roadmark(rm_solid)
lanesec4.add_left_lane(lane11)
lanesec4.add_left_lane(lane12)
lanesec4.add_right_lane(lane13)
lanesec4.add_right_lane(lane14)

# create the fifth and last lanesection with two left lanes and one right lane
lanesec5 = xodr.LaneSection(350.0 + 0.5 * transition_length, centerlane)
lane15 = xodr.Lane(a=lane_width)
lane15.add_roadmark(rm_dashed)
lane16 = xodr.Lane(a=lane_width)
lane16.add_roadmark(rm_solid)
lane17 = xodr.Lane(a=lane_width)
lane17.add_roadmark(rm_solid)
lanesec5.add_left_lane(lane15)
lanesec5.add_left_lane(lane16)
lanesec5.add_right_lane(lane17)

# create the lane links
lanelinker = xodr.LaneLinker()
lanelinker.add_link(predlane=lane1, succlane=lane4)
lanelinker.add_link(predlane=lane2, succlane=lane5)
lanelinker.add_link(predlane=lane3, succlane=lane7)
lanelinker.add_link(predlane=lane5, succlane=lane8)
lanelinker.add_link(predlane=lane6, succlane=lane9)
lanelinker.add_link(predlane=lane7, succlane=lane10)
lanelinker.add_link(predlane=lane8, succlane=lane12)
lanelinker.add_link(predlane=lane9, succlane=lane13)
lanelinker.add_link(predlane=lane10, succlane=lane14)
lanelinker.add_link(predlane=lane11, succlane=lane15)
lanelinker.add_link(predlane=lane12, succlane=lane16)
lanelinker.add_link(predlane=lane14, succlane=lane17)

# create the lanes
lanes = xodr.Lanes()

# add offset entries
lanes.add_laneoffset(xodr.LaneOffset(0.0, 0.0, 0.0, 0.0, 0.0))
lanes.add_laneoffset(xodr.LaneOffset(150.0 - 0.5 * transition_length, 0.0, 0.0, -c, -d))
lanes.add_laneoffset(xodr.LaneOffset(150.0 + 0.5 * transition_length, lane_width, 0.0, 0.0, 0.0))
lanes.add_laneoffset(xodr.LaneOffset(350.0 - 0.5 * transition_length, lane_width, 0.0, c, d))
lanes.add_laneoffset(xodr.LaneOffset(350.0 + 0.5 * transition_length, 0.0, 0.0, 0.0, 0.0))

# add links
lanes.add_lanesection(lanesec1, lanelinker)
lanes.add_lanesection(lanesec2, lanelinker)
lanes.add_lanesection(lanesec3, lanelinker)
lanes.add_lanesection(lanesec4, lanelinker)
lanes.add_lanesection(lanesec5, lanelinker)


# create the road
road = xodr.Road(1, planview, lanes)

# create the opendrive
odr = xodr.OpenDrive("myroad")
odr.add_road(road)

# adjust the roads and lanes
odr.adjust_roads_and_lanes()

# write the OpenDRIVE file as xodr using current script name
odr.write_xml(os.path.basename(__file__).replace(".py", ".xodr"))

# preview the road in esmini - adjust path to your file structure
from scenariogeneration import esmini
esmini(odr,os.path.join('/tmp/esmini'), car_density=5)
