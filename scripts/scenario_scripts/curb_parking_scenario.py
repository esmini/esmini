"""
Create simple parallel parking scenario using basic scenario triggers and actions

Assuming:
- current directory is esmini root, e.g. VSCode was started from there
- the curb_parking.xodr file has been created in resources/xodr by running create_curb_parking.py first
"""

import os, math
from scenariogeneration import xosc, esmini

# refer existing vehicle catalog and road
catalog = xosc.Catalog()
catalog.add_catalog("VehicleCatalog",'resources/xosc/Catalogs/Vehicles')
road = xosc.RoadNetwork(roadfile='resources/xodr/curb_parking.xodr')

# Create the storyboard to host both initialization and the story of more dynamic events
init = xosc.Init()
storyboard=xosc.StoryBoard(init,xosc.ValueTrigger('quit_trigger',3.0,xosc.ConditionEdge.none,xosc.SimulationTimeCondition(20.0, rule=xosc.Rule.greaterThan), triggeringpoint='stop'))

# add some vehicles to the scenario
entities = xosc.Entities()
entities.add_scenario_object("Ego", xosc.CatalogReference('VehicleCatalog', "car_white"))
entities.add_scenario_object("Target1", xosc.CatalogReference('VehicleCatalog', "car_red"))
entities.add_scenario_object("Target2", xosc.CatalogReference('VehicleCatalog', "car_blue"))
entities.add_scenario_object("Target3", xosc.CatalogReference('VehicleCatalog', "car_blue"))
init.add_init_action("Ego", xosc.TeleportAction(xosc.LanePosition(road_id=1, lane_id=-1, s=10, offset=0.0)))
init.add_init_action("Ego", xosc.AbsoluteSpeedAction(20.0/3.6, xosc.TransitionDynamics(xosc.DynamicsShapes.step,xosc.DynamicsDimension.time,0.0)))
init.add_init_action("Target1", xosc.TeleportAction(xosc.LanePosition(road_id=1, lane_id=-1, s=30, offset=0.0)))
init.add_init_action("Target1", xosc.AbsoluteSpeedAction(20.0/3.6, xosc.TransitionDynamics(xosc.DynamicsShapes.step,xosc.DynamicsDimension.time,0.0)))
init.add_init_action("Target2", xosc.TeleportAction(xosc.LanePosition(road_id=1, lane_id=1, s=60, offset=0.0)))
init.add_init_action("Target2", xosc.AbsoluteSpeedAction(20.0/3.6, xosc.TransitionDynamics(xosc.DynamicsShapes.step,xosc.DynamicsDimension.time,0.0)))
init.add_init_action("Target3", xosc.TeleportAction(xosc.LanePosition(road_id=1, lane_id=-2, s=83.0, offset=1.2, orientation=xosc.Orientation(h=3*math.pi/2, reference=xosc.ReferenceContext.relative))))

# Ego Story
story = xosc.Story("Story")
act = xosc.Act("Act")
story.add_act(act)
storyboard.add_story(story)
event1 = xosc.Event("EgoPark", xosc.Priority.override)
event1.add_trigger(xosc.EntityTrigger("EventStartTrigger", 0.0, xosc.ConditionEdge.none, xosc.DistanceCondition(1.0, "lessThan", xosc.LanePosition(25.0, 0.0, -1, 1)), "Ego"))
event1.add_action("LaneChangeAction", xosc.RelativeLaneChangeAction(-1, "Ego", xosc.TransitionDynamics(xosc.DynamicsShapes.cubic, xosc.DynamicsDimension.time, 2.5)))
event1.add_action("StopActionEgo", xosc.AbsoluteSpeedAction(0.0, xosc.TransitionDynamics(xosc.DynamicsShapes.linear, xosc.DynamicsDimension.rate, 1.35)))
maneuver_group = xosc.ManeuverGroup("maneuver_group").add_maneuver(xosc.Maneuver("maneuver").add_event(event1))
maneuver_group.add_actor("Ego")
act.add_maneuver_group(maneuver_group)

# create trajectory for target1
segment1 = xosc.ClothoidSplineSegment(
   curvature_start=0.0, curvature_end=-0.57, length=5.0,
   position_start=[xosc.RelativeObjectPosition("Target1", dx="0.0", dy="0.0")]
)

segment2 = xosc.ClothoidSplineSegment(
   curvature_start=-0.57, curvature_end=0.0, length=0.5
)

segments = [segment1, segment2]
clothoid_spline = xosc.ClothoidSpline(segments=segments)
trajectory = xosc.Trajectory("trajectory", False)
trajectory.add_shape(clothoid_spline)
trajectory_action = xosc.FollowTrajectoryAction(
   trajectory,
   xosc.FollowingMode.position,
   xosc.ReferenceContext.relative,
   1,
   0,
)
trajectory_event = xosc.Event("trajectory_event", xosc.Priority.overwrite)
trajectory_event.add_action("trajectory_action", trajectory_action)
trajectory_event.add_action("StopActionTarget1", xosc.AbsoluteSpeedAction(0.0, xosc.TransitionDynamics(xosc.DynamicsShapes.linear, xosc.DynamicsDimension.rate, 3.0)))
trajectory_event.add_trigger(xosc.EntityTrigger("TrajStartTrigger", 0.0, xosc.ConditionEdge.none, xosc.DistanceCondition(1.0, "lessThan", xosc.LanePosition(79.0, 0.0, -1, 1)), "Target1"))
maneuver_group2 = xosc.ManeuverGroup("maneuver_group2").add_maneuver(xosc.Maneuver("maneuver2").add_event(trajectory_event))
maneuver_group2.add_actor("Target1")
act.add_maneuver_group(maneuver_group2)

# write and preview the scenario
sce = xosc.Scenario('curb_parking','esmini team',xosc.ParameterDeclarations(),entities=entities,storyboard=storyboard,roadnetwork=road,catalog=catalog)
esmini(sce, ".", run_with_replayer=False, timestep=0.5,)