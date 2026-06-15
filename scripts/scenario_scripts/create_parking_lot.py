"""
scenariogeneration
https://github.com/pyoscx/scenariogeneration

This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.

Copyright (c) 2026 esmini team

Generate a parking lot

Script procedurally generates an OpenDRIVE description of a parking lot,
with a configurable (launch arguments) number of columns of parking spaces

Usage: python create_parking_lot.py --help

"""

import os
import math
import argparse
from scenariogeneration import ScenarioGenerator, prettyprint, xodr

class Scenario(ScenarioGenerator):
    def __init__(self, nr_parking_segments=10, nr_parking_spaces_per_segment=16):
        super().__init__()
        self.roads = []
        self.lanewidth = 3
        self.parking_space_length = 5
        self.parking_space_width = 2.5
        self.border_width = 10
        self.marking_width = 0.15
        self.segment_counter = 0
        self.nr_parking_segments = nr_parking_segments
        self.nr_parking_spaces_per_segment = nr_parking_spaces_per_segment
        self.odr = xodr.OpenDrive("my_road_network")

    def create_road(self, id, name, x, y, h, curvature, length, junction_id=-1, parking_spaces=False, add_border=False):

        planview = xodr.PlanView(x_start=x, y_start=y, h_start=h)
        if (curvature != 0):
            planview.add_geometry(xodr.Arc(curvature, length))
        else:
            planview.add_geometry(xodr.Line(length))

        # create centerlane
        centerlane = xodr.Lane(a=2)
        centerlane.add_roadmark(xodr.RoadMark(xodr.RoadMarkType.broken, length=1.0, space=1.0, width=0.15))
        lanesec = xodr.LaneSection(0, centerlane)

        # add driving lanes
        lanesec.add_left_lane(xodr.Lane(a=self.lanewidth))
        lanesec.add_right_lane(xodr.Lane(a=self.lanewidth))

        if (parking_spaces):
            # add parking area lanes
            lanesec.add_left_lane(xodr.Lane(a=self.parking_space_length, lane_type=xodr.LaneType.parking))
            lanesec.add_right_lane(xodr.Lane(a=self.parking_space_length, lane_type=xodr.LaneType.parking))

        if (add_border == 1):
            lanesec.add_left_lane(xodr.Lane(a=self.border_width, lane_type=xodr.LaneType.border))
        elif (add_border == -1):
            lanesec.add_right_lane(xodr.Lane(a=self.border_width, lane_type=xodr.LaneType.border))

        ## finalize the road
        lanes = xodr.Lanes()
        lanes.add_lanesection(lanesec)

        self.roads.append(xodr.Road(id, planview, lanes, name=name, road_type=junction_id))
        self.odr.add_road(self.roads[-1])

        return self.roads[-1]

    def create_parking_segment(self):
        base_idx = self.segment_counter * 11
        self.segment_counter += 1

        segment_length = self.nr_parking_spaces_per_segment * (self.parking_space_width - self.marking_width) + self.marking_width

        # create junctions and connections
        junction_north_id = 100 * (self.segment_counter)
        junction_north = xodr.Junction("junction_" + str(self.segment_counter) + "_north", junction_north_id)

        junction_south_id = 100 * (self.segment_counter) + 1
        junction_south = xodr.Junction("junction_" + str(self.segment_counter) + "_south", junction_south_id)


        # create the parking segment as a road with parking spaces on each side - idx = base_idx, id = base_id + 1
        self.create_road(base_idx + 1, "parking_segment_" + str(self.segment_counter), 0, self.segment_counter*(2*self.lanewidth + 2*self.parking_space_length),
                         0, 0, segment_length, junction_id=-1, parking_spaces=True)

        # create road at left north end - idx = base_idx + 1, id = base_idx + 2
        self.create_road(base_idx + 2, "left_north_" + str(self.segment_counter), segment_length + self.lanewidth, self.segment_counter*(2*self.lanewidth + 2*self.parking_space_length) + self.lanewidth,
                         math.pi/2, 0, self.parking_space_length, junction_id=-1, parking_spaces=False, add_border=-1)

        # create road at right north end - idx = base_idx + 2, id = base_idx + 3
        self.create_road(base_idx + 3, "right_north_" + str(self.segment_counter), segment_length + self.lanewidth, self.segment_counter*(2*self.lanewidth + 2*self.parking_space_length) - self.lanewidth,
                         -math.pi/2, 0, self.parking_space_length, junction_id=-1, parking_spaces=False, add_border=1)

        # create road at left south end - idx = base_idx + 3, id = base_idx + 4
        self.create_road(base_idx + 4, "left_south_" + str(self.segment_counter), -self.lanewidth, self.segment_counter*(2*self.lanewidth + 2*self.parking_space_length) - self.lanewidth,
                         -math.pi/2, 0, self.parking_space_length, junction_id=-1, parking_spaces=False, add_border=-1)

        # create road at right south end - idx = base_idx + 4, id = base_idx + 5
        self.create_road(base_idx + 5, "right_south_" + str(self.segment_counter), -self.lanewidth, self.segment_counter*(2*self.lanewidth + 2*self.parking_space_length) + self.lanewidth,
                         math.pi/2, 0, self.parking_space_length, junction_id=-1, parking_spaces=False, add_border=1)

        # create connecting road turning left at north end - idx = base_idx + 5, id = base_idx + 6
        self.create_road(base_idx + 6, "connecting_north_turn_left_" + str(self.segment_counter), segment_length, self.segment_counter*(2*self.lanewidth + 2*self.parking_space_length),
                         0, 1/self.lanewidth, self.lanewidth * math.pi / 2, junction_id=junction_north_id, parking_spaces=False)

        # create connecting road turning right at north end - idx = base_idx + 6, id = base_idx + 7
        self.create_road(base_idx + 7, "connecting_north_turn_right_" + str(self.segment_counter), segment_length, self.segment_counter*(2*self.lanewidth + 2*self.parking_space_length),
                         0, -1/self.lanewidth, self.lanewidth * math.pi / 2, junction_id=junction_north_id, parking_spaces=False)

        # create connecting road turning left at south end - idx = base_idx + 7, id = base_idx + 8
        self.create_road(base_idx + 8, "connecting_south_turn_left_" + str(self.segment_counter), 0, self.segment_counter*(2*self.lanewidth + 2*self.parking_space_length),
                         math.pi, 1/self.lanewidth, self.lanewidth * math.pi / 2, junction_id=junction_south_id, parking_spaces=False)

        # create connecting road turning right at south end - idx = base_idx + 8, id = base_idx + 9
        self.create_road(base_idx + 9, "connecting_south_turn_right_" + str(self.segment_counter), 0, self.segment_counter*(2*self.lanewidth + 2*self.parking_space_length),
                         math.pi, -1/self.lanewidth, self.lanewidth * math.pi / 2, junction_id=junction_south_id, parking_spaces=False)

        # create straight connecting road at north end - idx = base_idx + 9, id = base_idx + 10
        self.create_road(base_idx + 10,
                         "connecting_straight_north_" + str(self.segment_counter),
                         segment_length + self.lanewidth,
                         self.segment_counter*(2*self.lanewidth + 2*self.parking_space_length) - self.lanewidth,
                         math.pi/2, 0, 2*self.lanewidth,
                         junction_id=junction_north_id,
                         parking_spaces=False,
                         add_border=-1)

        # create straight connecting road at south end - idx = base_idx + 10, id = base_idx + 11
        self.create_road(base_idx + 11,
                         "connecting_straight_south_" + str(self.segment_counter),
                         -self.lanewidth,
                         self.segment_counter*(2*self.lanewidth + 2*self.parking_space_length) + self.lanewidth,
                         -math.pi/2, 0, 2*self.lanewidth,
                         junction_id=junction_south_id,
                         parking_spaces=False,
                         add_border=-1)

        con1 = xodr.Connection(self.roads[base_idx].id, self.roads[base_idx + 5].id, xodr.ContactPoint.start)
        con1.add_lanelink(-1, -1)
        con1.add_lanelink(1, 1)
        junction_north.add_connection(con1)

        con2 = xodr.Connection(self.roads[base_idx].id, self.roads[base_idx + 6].id, xodr.ContactPoint.start)
        con2.add_lanelink(-1, -1)
        con2.add_lanelink(1, 1)
        junction_north.add_connection(con2)

        con3 = xodr.Connection(self.roads[base_idx].id, self.roads[base_idx + 7].id, xodr.ContactPoint.start)
        con3.add_lanelink(1, -1)
        con3.add_lanelink(-1, 1)
        junction_south.add_connection(con3)

        con4 = xodr.Connection(self.roads[base_idx].id, self.roads[base_idx + 8].id, xodr.ContactPoint.start)
        con4.add_lanelink(1, -1)
        con4.add_lanelink(-1, 1)
        junction_south.add_connection(con4)

        con5 = xodr.Connection(self.roads[base_idx + 1].id, self.roads[base_idx + 9].id, xodr.ContactPoint.end)
        con5.add_lanelink(1, 1)
        con5.add_lanelink(-1, -1)
        junction_north.add_connection(con5)

        con6 = xodr.Connection(self.roads[base_idx + 2].id, self.roads[base_idx + 9].id, xodr.ContactPoint.start)
        con6.add_lanelink(1, -1)
        con6.add_lanelink(-1, 1)
        junction_north.add_connection(con6)

        con7 = xodr.Connection(self.roads[base_idx + 3].id, self.roads[base_idx + 10].id, xodr.ContactPoint.end)
        con7.add_lanelink(1, 1)
        con7.add_lanelink(-1, -1)
        junction_south.add_connection(con7)

        con8 = xodr.Connection(self.roads[base_idx + 4].id, self.roads[base_idx + 10].id, xodr.ContactPoint.start)
        con8.add_lanelink(1, -1)
        con8.add_lanelink(-1, 1)
        junction_south.add_connection(con8)

        self.roads[base_idx].add_successor(xodr.ElementType.junction, junction_north_id)
        self.roads[base_idx].add_predecessor(xodr.ElementType.junction, junction_south_id)

        self.roads[base_idx + 1].add_predecessor(xodr.ElementType.junction, junction_north_id)
        self.roads[base_idx + 2].add_predecessor(xodr.ElementType.junction, junction_north_id)

        self.roads[base_idx + 3].add_predecessor(xodr.ElementType.junction, junction_south_id)
        self.roads[base_idx + 4].add_predecessor(xodr.ElementType.junction, junction_south_id)

        self.roads[base_idx + 5].add_predecessor(xodr.ElementType.road, base_idx + 1, xodr.ContactPoint.end)
        self.roads[base_idx + 5].add_successor(xodr.ElementType.road, base_idx + 2, xodr.ContactPoint.start)

        self.roads[base_idx + 6].add_predecessor(xodr.ElementType.road, base_idx + 1, xodr.ContactPoint.end)
        self.roads[base_idx + 6].add_successor(xodr.ElementType.road, base_idx + 3, xodr.ContactPoint.start)

        self.roads[base_idx + 7].add_predecessor(xodr.ElementType.road, base_idx + 1, xodr.ContactPoint.start)
        self.roads[base_idx + 7].add_successor(xodr.ElementType.road, base_idx + 4, xodr.ContactPoint.start)

        self.roads[base_idx + 8].add_predecessor(xodr.ElementType.road, base_idx + 1, xodr.ContactPoint.start)
        self.roads[base_idx + 8].add_successor(xodr.ElementType.road, base_idx + 5, xodr.ContactPoint.start)

        self.roads[base_idx + 9].add_predecessor(xodr.ElementType.road, base_idx + 3, xodr.ContactPoint.start)
        self.roads[base_idx + 9].add_successor(xodr.ElementType.road, base_idx + 2, xodr.ContactPoint.start)

        self.roads[base_idx + 10].add_predecessor(xodr.ElementType.road, base_idx + 5, xodr.ContactPoint.start)
        self.roads[base_idx + 10].add_successor(xodr.ElementType.road, base_idx + 4, xodr.ContactPoint.start)

        self.odr.add_junction(junction_north)
        self.odr.add_junction(junction_south)

        if (self.segment_counter > 1):
            # connect with previous parking segment
            self.roads[base_idx + 2].add_successor(xodr.ElementType.road, self.roads[base_idx - 11 + 1].id, xodr.ContactPoint.end)
            self.roads[base_idx + 3].add_successor(xodr.ElementType.road, self.roads[base_idx - 11 + 4].id, xodr.ContactPoint.end)
            self.roads[base_idx - 11 + 1].add_successor(xodr.ElementType.road, self.roads[base_idx + 2].id, xodr.ContactPoint.end)
            self.roads[base_idx - 11 + 4].add_successor(xodr.ElementType.road, self.roads[base_idx + 3].id, xodr.ContactPoint.end)


    def road(self, **kwargs):

        # Add road network
        for i in range(self.nr_parking_segments):
            self.create_parking_segment()

        self.odr.adjust_roads_and_lanes()

        # Add parking spaces with markings
        for i in range(self.nr_parking_segments):
            base_idx = i * 11

            for j in range(self.nr_parking_spaces_per_segment):
                for side in [1, -1]:
                    park_obj = xodr.Object(
                                Type=xodr.ObjectType.parkingSpace,
                                id=1000 * i + j * 2 * 11 + j * 2 + side,
                                name="Parking_space_{}".format(id),
                                s=self.parking_space_width/2 + j * (self.parking_space_width - self.marking_width),
                                t=side * (self.lanewidth + self.parking_space_length/2),
                                zOffset=0.005,
                                orientation=xodr.Orientation.none,
                                length=self.parking_space_width - 2 * self.marking_width,
                                width=self.parking_space_length - self.marking_width,
                                height=3.0,
                                hdg=0 if side == -1 else math.pi,
                                pitch=0,
                                roll=0)

                    for side in [xodr.SideType.rear, xodr.SideType.right, xodr.SideType.front]:
                        park_marking = xodr.Marking(color="white", lineLength="10", spaceLength="0.0",
                                                    startOffset="0.0", stopOffset="0.0", width="{}".format(self.marking_width),
                                                    side=xodr.SideType(side), zOffset="0.005")
                        park_marking.add_userdata(xodr.UserData(code="lateralOffset", value=str(-self.marking_width/2)))
                        park_obj.add_marking(park_marking)

                    self.roads[base_idx].add_object(park_obj)

        return self.odr

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Create a parking lot OpenDRIVE network.")
    parser.add_argument("--nr_parking_segments", type=int, default=10,
                        help="Number of parking segments (default: 10)")
    parser.add_argument("--nr_parking_spaces_per_segment", type=int, default=16,
                        help="Number of parking spaces per segment (default: 16)")
    args = parser.parse_args()

    # uncomment the following lines to display the scenario using esmini
    # from scenariogeneration import esmini
    # esmini(Scenario(args.nr_parking_segments, args.nr_parking_spaces_per_segment),
    #        os.path.join('../esmini_demo'),
    #        car_density=(args.nr_parking_segments + args.nr_parking_spaces_per_segment) / 10)
