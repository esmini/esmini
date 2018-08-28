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
					carVector[i].setPosition(init.Actions.Private[i].Action[j].Position, "Lane");
				}
			}
		}
	}

	std::cout << "ScenarioEngine: initCarVector finished" << std::endl;
}



void ScenarioEngine::checkTimeHeadway(OSCCondition &condition)
{
	//// Number of triggering entities
	//unsigned triggeringEntitiesNum = condition.ByEntity.TriggeringEntities.Entity.size();

	//// objectIds of objects that can cause a trigg
	//std::vector<int> triggeringEntityIds(triggeringEntitiesNum);
	//std::vector<OSCPosition> triggeringEntityPos(triggeringEntitiesNum);

	//for (size_t i = 0; i < triggeringEntitiesNum; i++)
	//{
	//	triggeringEntityIds[i] = getObjectId(condition.ByEntity.TriggeringEntities.Entity[i].name);
	//	triggeringEntityPos[i] = carVector[triggeringEntityIds[i]].getPosition();
	//}

	////  objectId of objects that are measuaring from
	//int entityId = getObjectId(condition.ByEntity.EntityCondition.TimeHeadway.entity);
	//OSCPosition entityPos = carVector[entityId].getPosition();
	//double entitySpeed = carVector[entityId].getSpeed();

	//// Check which object are allowed to cause a trigg
	//std::vector<double> headwayTime(triggeringEntitiesNum);
	//std::vector<bool> triggs(triggeringEntitiesNum, false);


	//if (condition.ByEntity.EntityCondition.TimeHeadway.alongRoute == "true")
	//{
	//	
	//	// Take into account the dimensions of the vehicle
	//	if (condition.ByEntity.EntityCondition.TimeHeadway.freespace == "true")
	//	{
	//		// Go for road coordinates
	//		if (condition.ByEntity.EntityCondition.TimeHeadway.alongRoute == "true")
	//		{
	//			for (size_t i = 0; i < triggeringEntitiesNum; i++)
	//			{
	//				headwayTime[i] = (entityPos.Lane.s - triggeringEntityPos[i].Lane.s) / (entitySpeed/3.6);
	//				std::cout << "ScenarioEngine: headwayTime is " << headwayTime[i] << std::endl;

	//				if (condition.ByEntity.EntityCondition.TimeHeadway.rule == "greater_than")
	//				{

	//					if (headwayTime[i] > std::stod(condition.ByEntity.EntityCondition.TimeHeadway.value))
	//					{
	//						triggs[i] = true;
	//					}
	//				}
	//			}
	//		}
	//	}
	//}


	//if (condition.ByEntity.TriggeringEntities.rule == "any")
	//{
	//	for (size_t i = 0; i < triggeringEntitiesNum; i++)
	//	{
	//		if (triggs[i])
	//		{
	//			std::cout << "ScenarioEngine: " << condition.name << "has triggered" << std::endl;
	//			break;
	//		}
	//	}
	//}
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





	





