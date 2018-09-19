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

ScenarioGateway & ScenarioEngine::getScenarioGateway()
{
	return scenarioGateway;
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

void ScenarioEngine::initInit()
{
	std::cout << "ScenarioEngine: initStoryboard started" << std::endl;


	for (int i = 0; i < init.Actions.Private.size(); i++)
	{
		std::vector<std::string> actionEntities;
		actionEntities.push_back(init.Actions.Private[i].object);

		for (int j = 0; j < init.Actions.Private[i].Action.size(); j++)
		{

			OSCPrivateAction privateAction = init.Actions.Private[i].Action[j];
			std::vector<int> storyId{i,j};
			
			Action action(privateAction, cars, storyId, actionEntities);
			actions.addAction(action);
			actions.setStartAction(storyId, 0);
		}
	}

	actions.executeActions(0);

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
			Entities::ObjectStruct objectStruct = entities.Object[i];

			car.setObjectId(objectId);
			car.setName(objectName);
			car.setObjectStruct(objectStruct);

			for (size_t i = 0; i < objectStruct.Properties.size(); i++)
			{
				if (objectStruct.Properties[i].Property.name == "control")
				{
					if (objectStruct.Properties[i].Property.value == "external")
					{
						car.setExtControlled(true);
					}
				}
				else if (objectStruct.Properties[i].Property.name == "externalId")
				{
					car.setObjectId(std::stoi(objectStruct.Properties[i].Property.value));
				}

				// Add the car on the last run
				if (i == objectStruct.Properties.size()-1)
				{
					scenarioGateway.addExternalCar(car);
				}
			}
			cars.addCar(car);
		}
	}

	cars.addScenarioGateway(scenarioGateway);

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



