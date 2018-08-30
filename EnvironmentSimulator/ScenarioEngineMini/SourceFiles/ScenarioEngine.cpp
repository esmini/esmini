#include "ScenarioEngine.hpp"


ScenarioEngine::ScenarioEngine(Entities &entities, Init &init, std::vector<Story> &story, double startTime)
{
	std::cout << "ScenarioEngine: New ScenarioEngine created" << std::endl;

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


void ScenarioEngine::initCarVector()
{
	std::cout << "ScenarioEngine: initCarVector started" << std::endl;

	if (init.Actions.Private[0].exists)
	{

		for (size_t i = 0; i < init.Actions.Private.size(); i++)
		{			
			Car car;
			int objectId = i;
			car.setObjectId(objectId);
			car.setName(init.Actions.Private[i].object); // The names should be taken from entities instead.

			carVector.push_back(car);

			for (size_t j = 0; j < init.Actions.Private[i].Action.size(); j++)
			{

				// Speed
				if (!init.Actions.Private[i].Action[j].Longitudinal.Speed.Dynamics.shape.empty())
				{
					if (init.Actions.Private[i].Action[j].Longitudinal.Speed.Dynamics.shape == "step")
					{
						if (init.Actions.Private[i].Action[j].Longitudinal.Speed.Target.Absolute.value != NAN)
						{
							carVector[i].setSpeed(init.Actions.Private[i].Action[j].Longitudinal.Speed.Target.Absolute.value);
						}
					}
				}

				// Position
				else if (!init.Actions.Private[i].Action[j].Position.Lane.roadId.empty())
				{
					//carVector[i].setPosition(init.Actions.Private[i].Action[j].Position, "Lane");
					OSCPosition position = init.Actions.Private[i].Action[j].Position;

					int roadId = std::stoi(position.Lane.roadId);
					int laneId = position.Lane.laneId;
					double s = position.Lane.s;
					double offset = position.Lane.offset;

					carVector[i].setPosition(roadId, laneId, s, offset);
				}
			}
		}
	}

	std::cout << "ScenarioEngine: initCarVector finished" << std::endl;
}


int ScenarioEngine::getObjectId(std::string objectName)
{
	int objectId = -1;

	for (size_t i = 0; i < carVector.size(); i++)
	{
		if (carVector[i].getObjectName() == objectName)
		{
			objectId = carVector[i].getObjectId();
		}
	}

	return objectId;
}


void ScenarioEngine::printCarVector()
{
	for (size_t i = 0; i < carVector.size(); i++)
	{
		std::cout << "---------------------------------------" << std::endl;
		std::cout << "Print of carVector " << "\n" << std::endl;
		carVector[i].printState();

	}
}

void ScenarioEngine::stepObjects(double dt)
{
	for (size_t i = 0; i < carVector.size(); i++)
	{
		carVector[i].step(dt);
		//std::cout << "Car with actorId: " << carVector[i].getObjectId() << " stepped " << dt << " seconds"<< "\n" << std::endl;

	}
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
								OSCCondition condition = story[i].Act[j].Sequence[k].Maneuver[l].Event[m].StartConditions.ConditionGroup[n].Condition[o];

								if (!condition.ByEntity.EntityCondition.TimeHeadway.entity.empty())
								{
									TimeHeadway timeHeadway(condition, carVector, storyId, actionEntities);
									timeHeadwayVector.push_back(timeHeadway);
								}
							}
						}
						
						for (int n = 0; n < story[i].Act[j].Sequence[k].Maneuver[l].Event[m].Action.size(); n++)
						{
							
								OSCPrivateAction privateAction = story[i].Act[j].Sequence[k].Maneuver[l].Event[m].Action[n].Private;
								PrivateAction action(privateAction, carVector, storyId, actionEntities);
								actionVector.push_back(action);
						}
					}
				}
			}
		}
	}
}


void ScenarioEngine::checkConditions()
{
	bool trigged;

	for (size_t i = 0; i < timeHeadwayVector.size(); i++)
	{
		trigged = timeHeadwayVector[i].checkTimeHeadway();
		
		if (trigged)
		{

			for (size_t j = 0; j < actionVector.size(); j++)
			{
				if (timeHeadwayVector[i].storyId == actionVector[j].storyId)
				{
					actionVector[j].startAction = true;
				}
			}
			
			std::cout << "ScenarioEngine: " << timeHeadwayVector[i].condition.name << " is removed from timeHeadwayVector" << "\n" << std::endl;
			timeHeadwayVector.erase(timeHeadwayVector.begin() + i);
			
		}
	}
}


void ScenarioEngine::executeActions()
{
	for (size_t i = 0; i < actionVector.size(); i++)
	{
		if (actionVector[i].startAction)
		{
			if (actionVector[i].getFirstRun())
			{
				actionVector[i].setStartTime(simulationTime);
			}

			actionVector[i].ExecuteAction(simulationTime, timeStep);
		}

		if (actionVector[i].ActionCompleted)
		{
			std::cout << "ScenarioEngine: " << " privateAction is removed from actionVector" << "\n" << std::endl;
			actionVector.erase(actionVector.begin() + i);
		}
	}

}





	





