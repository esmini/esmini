import os
from scenariogeneration import xodr, esmini


# create the road
road = xodr.create_road(id=0, left_lanes=1, right_lanes=1, center_road_mark=xodr.std_roadmark_broken(), lane_width=4, geometry=[xodr.Line(20)])
odr = xodr.OpenDrive('objects')
odr.add_road(road)
odr.adjust_roads_and_lanes()

# add box
box = xodr.Object(s=5, t=-2, Type=xodr.ObjectType.building, length="3", width="1.8", hdg=0.7, zOffset=0, height=2)
road.add_object(box)

# add cylinder
cylinder = xodr.Object(s=12, t=-2, Type=xodr.ObjectType.obstacle, radius=1.2, zOffset=0, height=2)
road.add_object(cylinder)

# preview
esmini(odr, ".", car_density=0)
