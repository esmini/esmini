import os
from scenariogeneration import xodr, esmini


# create the road
road = xodr.create_road(id=0, left_lanes=0, right_lanes=1, center_road_mark=xodr.std_roadmark_broken(), lane_width=3.5, geometry=[xodr.Line(50)])
road.lanes.lanesections[0].add_left_lane(xodr.Lane(lane_type=xodr.LaneType.shoulder, a=6))
road.lanes.lanesections[0].add_right_lane(xodr.Lane(lane_type=xodr.LaneType.border, a=10))
odr = xodr.OpenDrive('objects')
odr.add_road(road)
odr.adjust_roads_and_lanes()

# create the trunk part of the tree as a cylinder, set radius != 0
# tree_trunk = xodr.Object(s=10, t=-7, Type=xodr.ObjectType.tree, radius=0.25, zOffset=0, height=5)
# road.add_object_roadside(tree_trunk, 15, 10, -7, xodr.RoadSide.right)

# create the top part of the tree, as an outline shape
tree = xodr.Object(s=10, t=-7, height=0, Type=xodr.ObjectType.tree, zOffset=0, hdg=2)
trunk_outline = xodr.Outline(id=1, closed=True)
trunk_outline.add_corner(xodr.CornerLocal(u=-0.2, v=-0.2, z=0, height=5, id=1))
trunk_outline.add_corner(xodr.CornerLocal(u=0.2, v=-0.2, z=0, height=5, id=2))
trunk_outline.add_corner(xodr.CornerLocal(u=0.2, v=0.2, z=0, height=5, id=3))
trunk_outline.add_corner(xodr.CornerLocal(u=-0.2, v=0.2, z=0, height=5, id=4))
tree.add_outline(trunk_outline)
top_outline = xodr.Outline(id=1, closed=True)
top_outline.add_corner(xodr.CornerLocal(u=3, v=0, z=5, height=4, id=1))
top_outline.add_corner(xodr.CornerLocal(u=-1.5, v=2.5, z=5, height=4, id=2))
top_outline.add_corner(xodr.CornerLocal(u=-1.5, v=-2.5, z=5, height=4, id=3))
tree.add_outline(top_outline)
# road.add_object(tree)
road.add_object_roadside(tree, 15, 10, -7, xodr.RoadSide.right)

# create parking space from bounding box
mark_width = 0.1  # parking marking width
parking_slot_bb = xodr.Object(s=12, t=4.65, Type=xodr.ObjectType.parkingSpace, length=5, width=2.5, height=3)

# for side in [xodr.SideType.left, xodr.SideType.rear, xodr.SideType.right, xodr.SideType.front]:
#     m = xodr.Marking(xodr.RoadMarkColor.white, 10, side, 0, 0, 0, width=mark_width)
#     m.add_userdata(xodr.UserData(code="lateralOffset", value=str(-mark_width/2)))
#     parking_slot_bb.add_marking(m)

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

m_front = xodr.Marking(xodr.RoadMarkColor.white, 10, xodr.SideType.front, 0, 0, 0, width=mark_width)
m_front.add_userdata(xodr.UserData(code="lateralOffset", value=str(-mark_width/2)))
parking_slot_bb.add_marking(m_front)

road.add_object(parking_slot_bb)

# create parking space from outline
parking_slot_ol = xodr.Object(s=17 + mark_width, t=4.65, Type=xodr.ObjectType.parkingSpace, length=5, width=2.5, height=3)
outline = xodr.Outline(id=1, closed=True)
outline.add_corner(xodr.CornerLocal(u=-2.5, v=-1.25, z=0, height=3, id=1))
outline.add_corner(xodr.CornerLocal(u=2.5, v=-1.25, z=0, height=3, id=2))
outline.add_corner(xodr.CornerLocal(u=2.5, v=1.25, z=0, height=3, id=3))
outline.add_corner(xodr.CornerLocal(u=-2.5, v=1.25, z=0, height=3, id=4))
parking_slot_ol.add_outline(outline)
m=xodr.Marking(xodr.RoadMarkColor.white, 10, xodr.SideType.front, 0, 0, 0, width=mark_width)
m.add_cornerReference(cornerReference=1)
m.add_cornerReference(cornerReference=2)
m.add_cornerReference(cornerReference=3)
m.add_cornerReference(cornerReference=4)
m.add_cornerReference(cornerReference=1)
m.add_userdata(xodr.UserData(code="lateralOffset", value=str(-mark_width/2)))
parking_slot_ol.add_marking(m)
road.add_object(parking_slot_ol)

esmini(odr, "/eknabe1/GIT/esmini_demo", car_density=0)
