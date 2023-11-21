
import math
from scenariogeneration import xodr
from scenariogeneration import xosc
from scenariogeneration import ScenarioGenerator
from enum import Enum

class Scenario(ScenarioGenerator):

    class Action1Modes(Enum):
        LANE_POSITION_REF_RELATIVE = 0
        LANE_POSITION_REF_ABSOLUTE = 1
        ROAD_POSITION_REF_RELATIVE = 2
        ROAD_POSITION_REF_ABSOLUTE = 3
        WORLD_POSITION = 4

    class Action2Modes(Enum):
        LANE_POSITION = 0
        ROAD_POSITION = 1
        LANE_POSITION_RELATIVE = 2
        ROAD_POSITION_RELATIVE = 3

    def __init__(self):

        ScenarioGenerator.__init__(self)
        self.naming = "numerical"
        self.n_headings = 8
        # self.road_angle = [0.5*math.pi]
        self.road_angle = [i * 2 * math.pi / float(self.n_headings) for i in range(self.n_headings)]
        # self.road_pitch = [0.55]
        self.road_pitch = [0, 0.55, -0.55]
        self.lane_width = 3.5
        self.action1_mode = self.Action1Modes.LANE_POSITION_REF_RELATIVE
        self.action2_mode = self.Action2Modes.ROAD_POSITION_RELATIVE
        self.road_length = 50.0
        # self.action2_mode = self.Action2Modes.LANE_POSITION_RELATIVE

    def road(self, **kwargs):

        odr = xodr.OpenDrive("Mulitple roads with various headings and slopes")
        id = 0
        for h in self.road_angle:
            for p in self.road_pitch:
                planView = xodr.PlanView(x_start = 10 * math.cos(h), y_start = 10 * math.sin(h), h_start = h)
                planView.add_geometry(xodr.Line(self.road_length))
                lanes = xodr.Lanes()
                centerLane = xodr.Lane(xodr.LaneType.driving, 0, 0, 0, 0, 0)
                centerLane.add_roadmark(xodr.RoadMark(xodr.RoadMarkType.broken, 0.15, 3, 1, 0, 0, xodr.MarkRule.none))
                laneSection = xodr.LaneSection(0, centerLane)
                rlane = xodr.Lane(xodr.LaneType.driving, self.lane_width, 0, 0, 0, 0)
                rlane.add_roadmark(xodr.RoadMark(xodr.RoadMarkType.solid, 0.15, 0, 0, 0, 0, xodr.MarkRule.none))
                llane = xodr.Lane(xodr.LaneType.driving, self.lane_width, 0, 0, 0, 0)
                llane.add_roadmark(xodr.RoadMark(xodr.RoadMarkType.solid, 0.15, 0, 0, 0, 0, xodr.MarkRule.none))
                laneSection.add_right_lane(rlane)
                laneSection.add_left_lane(llane)
                lanes.add_lanesection(laneSection)
                road = xodr.Road(id, planView, lanes)
                road.add_elevation(0, 0, -math.tan(p), 0, 0)
                odr.add_road(road)
                odr.adjust_roads_and_lanes()
                id += 1
        return odr

    def scenario(self, **kwargs):
        road = xosc.RoadNetwork(self.road_file)
        catalog = xosc.Catalog()
        catalog.add_catalog("VehicleCatalog", "../xosc/Catalogs/Vehicles")
        init = xosc.Init()
        entities = xosc.Entities()
        id = 0
        s_start = self.road_length - 5
        counter = 0

        act = xosc.Act("star_act")

        for h in self.road_angle:
            for p in self.road_pitch:
                for i in [-1, 1]:
                    objname = "car_" + str(id) + "_" + str(i)
                    entities.add_scenario_object(objname, xosc.CatalogReference("VehicleCatalog", "car_white" if i == -1 else "car_red"))

                    h_init = h
                    if i == 1:
                        h_init += math.pi

                    t = 0.5 * i * self.lane_width
                    h_init2 = h + 0.2 * (-1 if i == 1 else 1)
                    ds = 2.0
                    dt = 0.1

                    maneuver_group = xosc.ManeuverGroup("star_maneuver_group_" + objname, 1, False)
                    maneuver_group.add_actor(objname)
                    maneuver = xosc.Maneuver("star_maneuver_" + objname)

                    for j in range(8):  # range(7) / [0,6]
                        # create event
                        label = str(id) + "_" + str(i) + "_" + str(j)
                        trigger = xosc.ValueTrigger(
                            "time_trigger_" + label, 0.0, xosc.ConditionEdge.none, xosc.SimulationTimeCondition(dt + (j - 1) * dt, xosc.Rule.greaterOrEqual)
                        )

                        if j == 0:  # Init action, LanePosition, absolute heading, pitch omitted
                            init.add_init_action(objname, xosc.TeleportAction(xosc.LanePosition(s_start, 0, i, id, xosc.Orientation(h = h + math.pi, reference = xosc.ReferenceContext.absolute))))

                        elif j == 1:  # Event action, LanePosition, absolute heading, absolute pitch
                            event = xosc.Event("event_lane_position_abs_heading_pitch_" + label, xosc.Priority.override)
                            event.add_action("action_road_position_abs_heading_pitch_" +  label, xosc.TeleportAction(
                                xosc.LanePosition(s_start - ds * j, 0, i, id, xosc.Orientation(h = h + math.pi, p = -p, reference = xosc.ReferenceContext.absolute))))

                        elif j == 2:  # Event action, LanePosition, relative heading
                            event = xosc.Event("event_lane_position_rel_heading_" + label, xosc.Priority.override)
                            event.add_action("action_lane_position_rel_heading_" +  label, xosc.TeleportAction(
                                xosc.LanePosition(s_start - ds * j, 0, i, id, xosc.Orientation(h = math.pi if i == -1 else 0.0, reference = xosc.ReferenceContext.relative))))

                        elif j == 3:  # Event action, RoadPosition, absolute heading
                            event = xosc.Event("event_road_position_abs_heading_" + label, xosc.Priority.override)
                            event.add_action("action_road_position_abs_heading_" +  label, xosc.TeleportAction(
                                xosc.RoadPosition(s_start - ds * j, t, id, xosc.Orientation(h=h + math.pi, reference = xosc.ReferenceContext.absolute))))

                        elif j == 4:  # Event action, RoadPosition, relative heading
                            event = xosc.Event("event_road_position_rel_heading_" + label, xosc.Priority.override)
                            event.add_action("action_road_position_rel_heading_" +  label, xosc.TeleportAction(
                                xosc.RoadPosition(s_start - ds * j, t, id, xosc.Orientation(h = math.pi, reference = xosc.ReferenceContext.relative))))

                        elif j == 5:  # Event action, RoadPosition, relative heading
                            event = xosc.Event("event_road_position_rel_heading_" + label, xosc.Priority.override)
                            event.add_action("action_road_position_rel_heading_" +  label, xosc.TeleportAction(
                                xosc.RoadPosition(s_start - ds * j, t, id, xosc.Orientation(h = math.pi, reference = xosc.ReferenceContext.relative))))

                        elif j == 6:
                            if (i == -1):
                                event = xosc.Event("event_relative_road_position_ds_" + label, xosc.Priority.override)
                                event.add_action("action_relative_road_position_ds_" +  label, xosc.TeleportAction(
                                    xosc.RelativeRoadPosition(-0.5 * ds, 2 * t, "car_" + str(id) + "_1",  xosc.Orientation(h = math.pi, reference = xosc.ReferenceContext.relative))))
                            else:
                                event = xosc.Event("event_relative_road_position_ds_" + label, xosc.Priority.override)
                                event.add_action("action_relative_road_position_ds_" +  label, xosc.TeleportAction(
                                    xosc.RelativeRoadPosition(-1.5 * ds, 0.0, objname,  xosc.Orientation(h = math.pi, reference = xosc.ReferenceContext.relative))))

                        elif j == 7:
                            if (i == -1):
                                event = xosc.Event("event_relative_lane_position_ds_" + label, xosc.Priority.override)  # switch lanes
                                event.add_action("action_relative_lane_position_ds_" +  label, xosc.TeleportAction(
                                    xosc.RelativeLanePosition(0, "car_" + str(id) + "_1", 0, -0.5 * ds, orientation=xosc.Orientation(h = math.pi, reference = xosc.ReferenceContext.relative))))
                            else:
                                event = xosc.Event("event_relative_lane_position_ds_" + label, xosc.Priority.override)
                                event.add_action("action_relative_lane_position_ds_" +  label, xosc.TeleportAction(
                                    xosc.RelativeLanePosition(lane_id=1, entity=objname, offset=0.0, dsLane=1.5 * ds, orientation=xosc.Orientation(h = math.pi, reference = xosc.ReferenceContext.relative))))

                        if (j > 0):
                            event.add_trigger(trigger)
                            maneuver.add_event(event)

                    maneuver_group.add_maneuver(maneuver)
                    act.add_maneuver_group(maneuver_group)

                id += 1

        sb = xosc.StoryBoard(
            init,
            stoptrigger=xosc.ValueTrigger(
                "stop_trigger",
                0.1,
                xosc.ConditionEdge.none,
                xosc.StoryboardElementStateCondition(xosc.StoryboardElementType.act, "star_act", xosc.StoryboardElementState.endTransition),
                "stop",
            ),
        )
        sb.add_act(act)
        sce = xosc.Scenario(
            "Exercise positions in multiple road headings and slopes",
            "esmini team",
            xosc.ParameterDeclarations(),
            entities,
            sb,
            road,
            catalog,
        )

        return sce


if __name__ == "__main__":
    s = Scenario()

    s.generate_single("star_position_scenario")