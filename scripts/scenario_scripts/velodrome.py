import os
import math
from scenariogeneration import xodr, esmini

# settings
track_length = 2000.0
straight_length = 500.0
curve_radius = 125.0
bank_angle = 60.0  # degrees

# calculated values
arc_length = straight_length + 2 * math.pi * curve_radius - track_length / 2.0
clothoid_length = math.pi * curve_radius - arc_length

# create road as two identical segments connected in a loop
road1 = xodr.create_road(id=1, left_lanes=0, right_lanes=3, center_road_mark=xodr.std_roadmark_solid(), geometry=
    [*([
        xodr.Line(straight_length),
        xodr.Spiral(0.0, 1.0/curve_radius, clothoid_length),
        xodr.Arc(1.0/curve_radius, arc_length),
        xodr.Spiral(1.0/curve_radius, 0.0, clothoid_length)
    ]*2)]
)

# define coefficients for the superelevation (banking) polynomial along the clothoid parts
a = 0.0
b = 0.0
c = -3 * math.radians(bank_angle) / (clothoid_length ** 2)
d = 2 * math.radians(bank_angle) / (clothoid_length ** 3)

# add superelevation to the road, two identical segments along the road s coordinate
road1.add_superelevation(0.0, 0.0, 0.0, 0.0, 0.0)
for i in range(2):
   road1.add_superelevation((i + 1) * straight_length + (i * 2) * clothoid_length + i * arc_length, a, b, c, d)
   road1.add_superelevation((i + 1) * straight_length + (i * 2 + 1) * clothoid_length + i * arc_length, -math.radians(bank_angle), 0.0, 0.0, 0.0)
   road1.add_superelevation((i + 1) * straight_length + (i * 2 + 1) * clothoid_length + (i + 1) * arc_length, -math.radians(bank_angle), 0.0, -c, -d)
   road1.add_superelevation((i + 1) * straight_length + (i * 2 + 2) * clothoid_length + (i + 1) * arc_length, 0.0, 0.0, 0.0, 0.0)

# connect the road to itself to form a loop
road1.add_successor(xodr.ElementType.road, road1, xodr.ContactPoint.start)
road1.add_predecessor(xodr.ElementType.road, road1, xodr.ContactPoint.end)

# create the OpenDRIVE road file
odr = xodr.OpenDrive('velodrome')
odr.add_road(road1)
odr.adjust_roads_and_lanes()
odr.write_xml(os.path.basename(__file__).replace('.py','.xodr'))

# preview in esmini
esmini(odr, "/eknabe1/GIT/esmini_ps", car_density=5)
