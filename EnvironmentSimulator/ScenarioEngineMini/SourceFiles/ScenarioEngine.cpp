#include "ScenarioEngine.hpp"


ScenarioEngine::ScenarioEngine(Catalogs &catalogs, Entities &entities, Init &init, std::vector<Story> &story, double startTime)
{
	std::cout << "ScenarioEngine: New ScenarioEngine created" << std::endl;

	this->catalogs = catalogs;
	this->entities = entities;
	this->init = init;
	this->story = story;
	this->startTime = startTime;
}

void ScenarioEngine::setTimeStep(double timeStep)
{
	this->timeStep = timeStep;
}

void ScenarioEngine::setSimulationTime(double simulationTime)
{
	this->simulationTime = simulationTime;
}

void ScenarioEngine::printSimulationTime()
{
	std::cout << "ScenarioEngine: simulationTime = " << simulationTime << std::endl;
}

void ScenarioEngine::initRoute()
{
	if (!catalogs.RouteCatalog.Route.Waypoint.empty())
	{

		for (size_t i = 0; i < catalogs.RouteCatalog.Route.Waypoint.size(); i++)
		{
			roadmanager::Position position;

			// Lane position
			if (!catalogs.RouteCatalog.Route.Waypoint[i].Position->Lane.roadId.empty())
			{
				int roadId = std::stoi(catalogs.RouteCatalog.Route.Waypoint[i].Position->Lane.roadId);
				int lane_id = catalogs.RouteCatalog.Route.Waypoint[i].Position->Lane.laneId;
				double s = catalogs.RouteCatalog.Route.Waypoint[i].Position->Lane.s;
				double offset = catalogs.RouteCatalog.Route.Waypoint[i].Position->Lane.offset;

				position.SetLanePos(roadId, lane_id, s, offset);
			}

			route.setName(catalogs.RouteCatalog.Route.name);
			route.AddWaypoint(&position);
		}
	}
}

void ScenarioEngine::initStoryboard()
{
	std::cout << "ScenarioEngine: initStoryboard started" << std::endl;

	if (!init.Actions.Private.empty())
	{

		for (size_t i = 0; i < init.Actions.Private.size(); i++)
		{
			std::string objectName = init.Actions.Private[i].object;

			for (size_t j = 0; j < init.Actions.Private[i].Action.size(); j++)
			{

				// Speed
				if (!init.Actions.Private[i].Action[j].Longitudinal.Speed.Dynamics.shape.empty())
				{
					if (init.Actions.Private[i].Action[j].Longitudinal.Speed.Dynamics.shape == "step")
					{
						if (init.Actions.Private[i].Action[j].Longitudinal.Speed.Target.Absolute.value != NAN)
						{
							double speed = init.Actions.Private[i].Action[j].Longitudinal.Speed.Target.Absolute.value;
							cars.setSpeed(objectName, speed);
						}
					}
				}

				// Lane position
				else if (!init.Actions.Private[i].Action[j].Position.Lane.roadId.empty())
				{
					OSCPosition position = init.Actions.Private[i].Action[j].Position;

					int roadId = std::stoi(position.Lane.roadId);
					int laneId = position.Lane.laneId;
					double s = position.Lane.s;
					double offset = (std::isnan(position.Lane.offset)) ? 0 : std::isnan(position.Lane.offset);

					roadmanager::Position pos(roadId, laneId, s, offset);
					cars.setPosition(objectName, pos);
				}

				// Route position
				else if (!init.Actions.Private[i].Action[j].Position.Route.RouteRef.CatalogReference.catalogName.empty())
				{
					std::string routeEntryName = init.Actions.Private[i].Action[j].Position.Route.RouteRef.CatalogReference.entryName;

					// This doesnt make sense but the route is defined inline which is not allowed...
					// Guess we will need to have multiple routes what we look through
					if (routeEntryName == route.getName())
					{
						// Position according to the init
						double pathS = init.Actions.Private[i].Action[j].Position.Route.Position.LaneCoord.pathS;
						double laneId = init.Actions.Private[i].Action[j].Position.Route.Position.LaneCoord.laneId;
						double laneOffset = init.Actions.Private[i].Action[j].Position.Route.Position.LaneCoord.laneOffset;

						//roadmanager::Position routePosition = route.GetPosition(pathS);	// Would like such a function that returns a roadmanager::Position according to how the route i specified.
						//
						//// Position according to the route
						//double routeRoadId = routePosition.GetTrackId();
						//double routeS = routePosition.GetS();
						//double routeOffset = routePosition.GetOffset();

						//// Cars position
						//roadmanager::Position pos(routeRoadId, laneId, routeS, laneOffset + routeOffset);
						//cars.setPosition(objectName, pos);
					}
				}

				// Meeting
				else if (!init.Actions.Private[i].Action[j].Meeting.mode.empty())
				{

				}
			}
		}
	}

	std::cout << "ScenarioEngine: initStoryboard finished" << std::endl;
}

void ScenarioEngine::initCars()
{
	std::cout << "ScenarioEngine: initCars started" << std::endl;

	if (!entities.Object.empty())
	{
		for (size_t i = 0; i < entities.Object.size(); i++)
		{
			Car car;
			int objectId = i;
			std::string objectName = entities.Object[i].name;

			car.setObjectId(objectId);
			car.setName(objectName);
			cars.addCar(car);
		}
	}

	std::cout << "ScenarioEngine: initCars finished" << std::endl;
}

void ScenarioEngine::printCars()
{
	cars.printCar();
}

void ScenarioEngine::stepObjects(double dt)
{
	cars.step(dt);
}

void ScenarioEngine::initConditions()
{

	for (int i = 0; i < story.size(); i++)
	{
		for (int j = 0; j < story[i].Act.size(); j++)
		{
			for (int k = 0; k < story[i].Act[j].Sequence.size(); k++)
			{
				std::vector<std::string> actionEntities;

				for (int l = 0; l < story[i].Act[j].Sequence[k].Actors.Entity.size(); l++)
				{
					actionEntities.push_back(story[i].Act[j].Sequence[k].Actors.Entity.back().name);
				}

				for (int l = 0; l < story[i].Act[j].Sequence[k].Maneuver.size(); l++)
				{
					for (int m = 0; m < story[i].Act[j].Sequence[k].Maneuver[l].Event.size(); m++)
					{
						std::vector<int> storyId;
						storyId = {i,j,k,l,m};

						for (int n = 0; n < story[i].Act[j].Sequence[k].Maneuver[l].Event[m].StartConditions.ConditionGroup.size(); n++)
						{
							for (int o = 0; o < story[i].Act[j].Sequence[k].Maneuver[l].Event[m].StartConditions.ConditionGroup[n].Condition.size(); o++)
							{
								OSCCondition oscCondition = story[i].Act[j].Sequence[k].Maneuver[l].Event[m].StartConditions.ConditionGroup[n].Condition[o];
								Condition condition(oscCondition, cars, storyId, actionEntities);
								conditions.addCondition(condition);
							}
						}
						
						for (int n = 0; n < story[i].Act[j].Sequence[k].Maneuver[l].Event[m].Action.size(); n++)
						{
								OSCPrivateAction oscPrivateAction = story[i].Act[j].Sequence[k].Maneuver[l].Event[m].Action[n].Private;
								Action action(oscPrivateAction, cars, storyId, actionEntities);
								actions.addAction(action);
						}
					}
				}
			}
		}
	}
}

void ScenarioEngine::checkConditions()
{
	bool triggeredCondition = conditions.checkConditions();

	if (triggeredCondition)
	{
		std::vector<int> lastTriggeredStoryId = conditions.getLastTriggeredStoryId();
		actions.setStartAction(lastTriggeredStoryId, simulationTime);
	}

}

void ScenarioEngine::executeActions()
{
	actions.executeActions(simulationTime);
}



