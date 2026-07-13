import os
import math
from scenariogeneration import xodr, esmini

# tuneable parameters
road_length = 100
lane_width = 3
parking_space_width = 2.5
parking_space_length = 5
transition_length = 10
nr_parking_spaces = 5
parking_1_start_s = 10
mark_width = 0.1
curb_width = 0.15

# cubic coefficients for lane width transition
b = 0  # tangent aligned with road s-axis at start and end of transitions
c = 3 * parking_space_width / (transition_length ** 2)  # coefficient for quadratic term
d = -2 * parking_space_width / (transition_length ** 3) # coefficient for cubic term

road = xodr.create_road(xodr.Line(road_length), id=1, left_lanes=1, right_lanes=0, lane_width=3, center_road_mark=xodr.std_roadmark_broken())

# add custom lanes on right side including the parking area
road.lanes.lanesections[0].add_right_lane(xodr.Lane(lane_type=xodr.LaneType.driving, a=3))
road.lanes.lanesections[0].add_right_lane(xodr.Lane(lane_type=xodr.LaneType.parking, a=0))
road.lanes.lanesections[0].add_right_lane(xodr.Lane(lane_type=xodr.LaneType.curb, a=curb_width))

# define the lane widths to form the additional parking area 1
s = 0
road.lanes.lanesections[0].rightlanes[1].add_lane_width(a=0, b=0, c=0, d=0, soffset=s)
s += parking_1_start_s
road.lanes.lanesections[0].rightlanes[1].add_lane_width(a=0, b=0, c=c, d=d, soffset=s)
s += transition_length
road.lanes.lanesections[0].rightlanes[1].add_lane_width(a=parking_space_width, b=0, c=0, d=0, soffset=s)
s += nr_parking_spaces * (parking_space_length - mark_width)
road.lanes.lanesections[0].rightlanes[1].add_lane_width(a=parking_space_width, b=0, c=-c, d=-d, soffset=s)
s += transition_length
road.lanes.lanesections[0].rightlanes[1].add_lane_width(a=0, b=0, c=0, d=0, soffset=s)
road.lanes.lanesections[0].rightlanes[2].add_height(0.1)

odr = xodr.OpenDrive('objects')
odr.add_road(road)
odr.adjust_roads_and_lanes()

# create parking spaces 1 from bounding box
for i in range(nr_parking_spaces):
   parking_slot_bb = xodr.Object(
      s = i*parking_space_length + parking_1_start_s + transition_length + parking_space_length/2 + mark_width,
      t = -(lane_width+parking_space_width/2+mark_width/2),
      Type=xodr.ObjectType.parkingSpace,
      length=5-mark_width,
      width=2.5-2*mark_width,
      height=0.1)

   # unroll loop for more control over the individual markings
   m_front = xodr.Marking(xodr.RoadMarkColor.white, 10, xodr.SideType.front, 0, 0, 0, width=mark_width)
   m_front.add_userdata(xodr.UserData(code="lateralOffset", value=str(-mark_width/2)))
   parking_slot_bb.add_marking(m_front)

   m_left = xodr.Marking(xodr.RoadMarkColor.white, 10, xodr.SideType.left, 0, 0, 0, width=mark_width)
   m_left.add_userdata(xodr.UserData(code="lateralOffset", value=str(-mark_width/2)))
   parking_slot_bb.add_marking(m_left)

   m_rear = xodr.Marking(xodr.RoadMarkColor.white, 10, xodr.SideType.rear, 0, 0, 0, width=mark_width)
   m_rear.add_userdata(xodr.UserData(code="lateralOffset", value=str(-mark_width/2)))
   parking_slot_bb.add_marking(m_rear)

   # road.add_object_roadside(parking_slot_bb, 5, parking_start_s+transition_length+parking_space_length/2+mark_width/2, 0, xodr.RoadSide.right)
   road.add_object(parking_slot_bb)

# define the lane widths to form the additional parking area 2
parking_2_start_s = s + 20
ds = 0.01
s = parking_2_start_s
road.lanes.lanesections[0].rightlanes[1].add_lane_width(a=0, b=0, c=0, d=0, soffset=s)
road.lanes.lanesections[0].rightlanes[1].add_lane_width(a=parking_space_length, b=0, c=0, d=0, soffset=s)
s += nr_parking_spaces * (parking_space_width - mark_width) + mark_width + curb_width/2
road.lanes.lanesections[0].rightlanes[1].add_lane_width(a=parking_space_length, b=0, c=0, d=0, soffset=s)
road.lanes.lanesections[0].rightlanes[1].add_lane_width(a=0, b=0, c=0, d=0, soffset=s)

curb1 = xodr.Object(
   s = parking_2_start_s - curb_width/2,
   t = -(lane_width+parking_space_length/2 + curb_width/2),
   Type=xodr.ObjectType.barrier,
   length=curb_width,
   width=parking_space_length + curb_width,
   height=0.1)
road.add_object(curb1)

curb2 = xodr.Object(
   s = parking_2_start_s + nr_parking_spaces * (parking_space_width - mark_width) + mark_width + curb_width/2,
   t = -(lane_width+parking_space_length/2 + curb_width/2),
   Type=xodr.ObjectType.barrier,
   length=curb_width,
   width=parking_space_length + curb_width,
   height=0.1)
road.add_object(curb2)

# create parking spaces 2 from bounding box
s = parking_2_start_s + parking_space_width/2
for i in range(nr_parking_spaces):
   parking_slot_bb = xodr.Object(
      s = s,
      t = -(lane_width+parking_space_length/2+mark_width/2),
      Type=xodr.ObjectType.parkingSpace,
      length=5-mark_width,
      width=2.5-2*mark_width,
      height=0.1,
      hdg=-math.pi/2)
   s += parking_space_width - mark_width

   # unroll loop for more control over the individual markings
   m_left = xodr.Marking(xodr.RoadMarkColor.white, 10, xodr.SideType.left, 0, 0, 0, width=mark_width)
   m_left.add_userdata(xodr.UserData(code="lateralOffset", value=str(-mark_width/2)))
   parking_slot_bb.add_marking(m_left)

   m_rear = xodr.Marking(xodr.RoadMarkColor.white, 10, xodr.SideType.rear, 0, 0, 0, width=mark_width)
   m_rear.add_userdata(xodr.UserData(code="lateralOffset", value=str(-mark_width/2)))
   parking_slot_bb.add_marking(m_rear)

   m_right = xodr.Marking(xodr.RoadMarkColor.white, 10, xodr.SideType.right, 0, 0, 0, width=mark_width)
   m_right.add_userdata(xodr.UserData(code="lateralOffset", value=str(-mark_width/2)))
   parking_slot_bb.add_marking(m_right)

   # road.add_object_roadside(parking_slot_bb, 5, parking_start_s+transition_length+parking_space_length/2+mark_width/2, 0, xodr.RoadSide.right)
   road.add_object(parking_slot_bb)


esmini(odr, "/eknabe1/GIT/esmini_ps", car_density=0)
