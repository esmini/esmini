#include "PrivateAction.hpp"



PrivateAction::PrivateAction(OSCPrivateAction &privateAction, std::vector<Car> &carVector, std::vector<int> storyId, std::vector<std::string> &actionEntities)
{
	this->privateAction = privateAction;
	carVectorPtr = &carVector;
	this->actionEntities = actionEntities;
	this->storyId = storyId;

	firstRun = true;
	ActionCompleted = false;
	startAction = false;
	 
	for (size_t i = 0; i < actionEntities.size(); i++)
	{
		actionEntitiesIds.push_back(getObjectId(actionEntities[i]));
	}

	// Identify actionType
	if (privateAction.Lateral.LaneChange.Dynamics.shape == "sinusoidal")
	{
		if (!isnan(privateAction.Lateral.LaneChange.Dynamics.time))
		{
			actionType = "sinusoidal-time";
		}
	}


	if (!isnan(privateAction.Longitudinal.Speed.Dynamics.rate)) // Should be (!...Dynamics.shape.empty()) Wrong in osc
	{
		actionType = "speed";
	}
}


void PrivateAction::setStartTime(double simulationTime)
{
	startTime = simulationTime;
}


bool PrivateAction::getFirstRun()
{
	return firstRun;
}


void PrivateAction::ExecuteAction(double simulationTime, double timeStep) {

	firstRun = false;

	if (actionType == "sinusoidal-time")
	{
		executeSinusoidal(simulationTime);
	}

	if (actionType == "speed")
	{
		executeSpeed(simulationTime, timeStep);
	}
}


void PrivateAction::executeSinusoidal(double simulationTime)
{
	double time = privateAction.Lateral.LaneChange.Dynamics.time;
	double n = 1;
	double f = 3.1415 * n / time;

	for (size_t i = 0; i < actionEntitiesIds.size(); i++)
	{		
		double initialOffset = 0; // = carVectorPtr[actionEntitiesIds[i]].getOffset(); This will accumulate. Will need original offset.
		double A = 1; // TODO: Should be distance between lanes + possible initial offset
		double newOffset = initialOffset + (A * cos((startTime - simulationTime) * f) - 1);

		if ((*carVectorPtr)[actionEntitiesIds[i]].getPositionType() == "Lane")
		{
			(*carVectorPtr)[actionEntitiesIds[i]].setOffset(newOffset);
		}
	}
	
	// Could switch lane after time/2 and swap sign of the offset

	// Should use the target position instead of time to decide when action is completed
	if (simulationTime >= startTime + time)
	{
		ActionCompleted = true;
		startAction = false;
	}
	
}


void PrivateAction::executeSpeed(double simulationTime, double timeStep)
{
	if (privateAction.Longitudinal.Speed.Dynamics.rate != NAN)
	{
		for (size_t i = 0; i < actionEntitiesIds.size(); i++)
		{

			double newSpeed = (*carVectorPtr)[actionEntitiesIds[i]].getSpeed() + privateAction.Longitudinal.Speed.Dynamics.rate*timeStep;

			(*carVectorPtr)[actionEntitiesIds[i]].setSpeed(newSpeed);


			if (privateAction.Longitudinal.Speed.Target.Absolute.value != NAN)
			{
				if (privateAction.Longitudinal.Speed.Target.Absolute.value >= (*carVectorPtr)[actionEntitiesIds[i]].getSpeed())
				{
					ActionCompleted = true;
					startAction = false;
				}
			}
		}
	}
	

}


int PrivateAction::getObjectId(std::string objectName)
{
	int objectId = -1;

	for (size_t i = 0; i < (*carVectorPtr).size(); i++)
	{
		if ((*carVectorPtr)[i].getObjectName() == objectName)
		{
			objectId = (*carVectorPtr)[i].getObjectId();
		}
	}

	return objectId;
}

//PrivateAction::~PrivateAction()
//{
//}
