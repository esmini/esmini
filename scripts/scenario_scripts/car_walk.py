
import os
from scenariogeneration import xosc, prettyprint

### locate catalogs
esmini_path = 'c:/eknabe1/GIT/esmini_dev'
catalog = xosc.Catalog()
catalog.add_catalog("VehicleCatalog",'/tmp/openx-assets/catalogs')

### refer road
road = xosc.RoadNetwork(roadfile=esmini_path+'/resources/xodr/straight_500m.xodr', scenegraph=esmini_path+'/resources/models/straight_500m.osgb')

## create entities
entities = xosc.Entities()
cars = [
    ['car1', 'm1_volvo_v60_polestar_2013'],
    ['car2', 'm1_dacia_duster_2010'],
    ['car3', 'n1_fiat_ducato_2014'],
    ['car4', 'm1_bmw_x1_2016'],
    ['car5', 'm1_audi_tt_2014_roadster'],
    ['car6', 'm1_mercedes_sl65amg_2008'],
    ['car7', 'n2_gmc_hummer_2021_pickup'],
    ['car8', 'm1_hyundai_tucson_2015'],
    ['car9', 'm1_volvo_ex30_2024'],
    ['car10', 'n2_tesla_cybertruck_2024'],
    ['car11', 'm1_audi_q7_2015'],
    ['car12', 'm1_mini_countryman_2016'],
]

### create init and storyboard
init = xosc.Init()
storyboard=xosc.StoryBoard(init,xosc.ValueTrigger('quit_trigger',3.0,xosc.ConditionEdge.none,xosc.StoryboardElementStateCondition(xosc.StoryboardElementType.story, 'Story',xosc.StoryboardElementState.completeState), triggeringpoint='stop'))
story = xosc.Story("Story")
act = xosc.Act("Act")
story.add_act(act)
storyboard.add_story(story)

s = 260  # use for positioning the vehicles along the road
for car in cars:
    entities.add_scenario_object(car[0], xosc.CatalogReference('openx_assets_3d.catalog', car[1]))
    init.add_init_action(car[0], xosc.TeleportAction(xosc.LanePosition(road_id=1, lane_id=1, s=s, offset=0.0)))
    init.add_init_action(car[0], xosc.AbsoluteSpeedAction(2.5, xosc.TransitionDynamics(xosc.DynamicsShapes.step,xosc.DynamicsDimension.time,0.0)))

    event1 = xosc.Event("TargetStart", xosc.Priority.override)
    event1.add_trigger(xosc.EntityTrigger("EventStartTrigger", 0.0, xosc.ConditionEdge.none, xosc.ReachPositionCondition(xosc.LanePosition(255, 0.0, 1, 1), 1.0), car[0]))
    event1.add_action("TargetLaneChangeAction", xosc.AbsoluteLaneChangeAction(-1, xosc.TransitionDynamics(xosc.DynamicsShapes.cubic, xosc.DynamicsDimension.time, 5.0)))
    maneuver_group = xosc.ManeuverGroup("maneuver_group").add_maneuver(xosc.Maneuver("maneuver").add_event(event1))
    maneuver_group.add_actor(car[0])
    act.add_maneuver_group(maneuver_group)

    s += 10

## create the scenario
sce = xosc.Scenario('car_walk, see https://esmini.github.io/#_using_openx_assets_library for more info','esmini team',xosc.ParameterDeclarations(),entities=entities,storyboard=storyboard,roadnetwork=road,catalog=catalog)

# write the OpenSCENARIO file as xosc using current script name
sce.write_xml(os.path.basename(__file__).replace('.py','.xosc'))

# uncomment the following lines to display the scenario using esmini
from scenariogeneration import esmini
esmini(sce, esmini_path, run_with_replayer=False, timestep=0.05, resource_path='/tmp/openx-assets/model3d')
