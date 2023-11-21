
import math
from scenariogeneration import xodr
from scenariogeneration import xosc, prettyprint
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
        # self.action2_mode = self.Action2Modes.LANE_POSITION_RELATIVE

    def road(self, **kwargs):

        odr = xodr.OpenDrive("Mulitple roads with various headings and slopes")
        id = 0
        for h in self.road_angle:
            for p in self.road_pitch:
                planView = xodr.PlanView(x_start = 10 * math.cos(h), y_start = 10 * math.sin(h), h_start = h)
                planView.add_geometry(xodr.Line(50))
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
        s_start = 35
        counter = 0

        for h in self.road_angle:
            for p in self.road_pitch:
                for i in [-1, 1]:
                    egoname = "Ego_" + str(id) + "_" + str(i)
                    entities.add_scenario_object(egoname, xosc.CatalogReference("VehicleCatalog", "car_white"))

                    h_init = h
                    if i == 1:
                        h_init += math.pi

                    t = 0.5 * i * self.lane_width
                    h_init2 = h + 0.2 * (-1 if i == 1 else 1)

                    if self.action1_mode == self.Action1Modes.LANE_POSITION_REF_ABSOLUTE:
                        init.add_init_action(
                            egoname, xosc.TeleportAction(xosc.LanePosition(s_start, 0, i, id, xosc.Orientation(h=h_init, p=p, reference = xosc.ReferenceContext.absolute)))
                        )
                    elif self.action1_mode == self.Action1Modes.ROAD_POSITION_REF_ABSOLUTE:
                        init.add_init_action(
                            egoname, xosc.TeleportAction(xosc.RoadPosition(s_start, t, id, xosc.Orientation(h=h_init, p=p, reference = xosc.ReferenceContext.absolute)))
                        )
                    elif self.action1_mode == self.Action1Modes.LANE_POSITION_REF_RELATIVE:
                        init.add_init_action(
                            egoname, xosc.TeleportAction(xosc.LanePosition(s_start, 0, i, id, xosc.Orientation(h=0, p=0, reference = xosc.ReferenceContext.relative)))
                        )
                    elif self.action1_mode == self.Action1Modes.ROAD_POSITION_REF_RELATIVE:
                        init.add_init_action(
                            egoname, xosc.TeleportAction(xosc.RoadPosition(s_start, t, id, xosc.Orientation(h=0, p=0, reference = xosc.ReferenceContext.relative)))
                        )
                    elif self.action1_mode == self.Action1Modes.WORLD_POSITION:
                        init.add_init_action(
                            egoname, xosc.TeleportAction(xosc.WorldPosition(
                                x = s_start * math.cos(h_init2) - t * math.sin(h_init2),
                                y = s_start * math.sin(h_init2) + t * math.cos(h_init2),
                                z = s_start * math.tan(p))
                            )
                        )
                    else:
                        print("unexpected action1_mode: ", self.action1_mode)
                        exit

                    if self.action2_mode == self.Action2Modes.LANE_POSITION:
                        init.add_init_action(
                            egoname, xosc.TeleportAction(xosc.LanePosition(s_start, 0, i, id, xosc.Orientation(h=h_init, p=p, reference = xosc.ReferenceContext.absolute)))
                        )
                    elif self.action2_mode == self.Action2Modes.ROAD_POSITION:
                        init.add_init_action(
                            egoname, xosc.TeleportAction(xosc.RoadPosition(s_start, t, id, xosc.Orientation(h=0.0, p=0, reference = xosc.ReferenceContext.relative)))
                        )
                    elif self.action2_mode == self.Action2Modes.LANE_POSITION_RELATIVE:
                        init.add_init_action(
                            egoname, xosc.TeleportAction(xosc.RelativeLanePosition(0, egoname, 0.0, ds = 0.0, orientation = xosc.Orientation(h=math.pi, p=0, reference = xosc.ReferenceContext.relative)))
                        )
                    elif self.action2_mode == self.Action2Modes.ROAD_POSITION_RELATIVE:
                        init.add_init_action(
                            egoname, xosc.TeleportAction(xosc.RelativeRoadPosition(0.0, 0, egoname, xosc.Orientation(h=math.pi, p=0.0, r=0.0, reference = xosc.ReferenceContext.relative)))
                        )
                    else:
                        print("unexpected action2_mode: ", self.action2_mode)
                        exit

                    init.add_init_action(egoname, xosc.AbsoluteSpeedAction(2.0, xosc.TransitionDynamics(xosc.DynamicsShapes.step, xosc.DynamicsDimension.time, 1)))

                    init.add_init_action(
                        egoname, xosc.AbsoluteLaneChangeAction(-i, xosc.TransitionDynamics(xosc.DynamicsShapes.linear, xosc.DynamicsDimension.time, value=5.0), 0.0)
                    )

                id += 1

        sb = xosc.StoryBoard(
            init,
            stoptrigger=xosc.ValueTrigger(
                "stop_trigger",
                0,
                xosc.ConditionEdge.none,
                xosc.SimulationTimeCondition(13.0, xosc.Rule.greaterThan),
                "stop",
            ),
        )
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

    s.generate("star_scenario")