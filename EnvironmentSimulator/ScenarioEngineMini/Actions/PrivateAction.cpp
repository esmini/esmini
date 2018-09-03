#include "PrivateAction.hpp"



PrivateAction::PrivateAction(OSCPrivateAction &privateAction, Cars &cars, std::vector<int> storyId, std::vector<std::string> &actionEntities)
{
	this->privateAction = privateAction;
	carsPtr = &cars;
	this->actionEntities = actionEntities;
	this->storyId = storyId;

	firstRun = true;
	ActionCompleted = false;
	startAction = false;
	 
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
	double f = 3.1415 / time;

	std::string targetObject = privateAction.Lateral.LaneChange.Target.Relative.object;
	double targetValue= privateAction.Lateral.LaneChange.Target.Relative.value;
	double currentLane = (*carsPtr).getPosition(targetObject).GetLaneId();

	double targetLane = currentLane + targetValue;

	// targetLane may become 0:
	if (targetLane == 0 && currentLane > 0)
	{
		targetLane = -1;
	}
	else if (targetLane == 0 && currentLane < 0)
	{
		targetLane = 1;
	}

	for (size_t i = 0; i < actionEntities.size(); i++)
	{
		roadmanager::Position position = (*carsPtr).getPosition(actionEntities[i]);
		roadmanager::Road *road = position.GetOpenDrive()->GetRoadById(position.GetTrackId());
		roadmanager::LaneSection *lanesection = road->GetLaneSectionByS(position.GetS());

		double width = lanesection->GetWidthBetweenLanes(
			(*carsPtr).getPosition(actionEntities[i]).GetLaneId(),
			targetLane,
			position.GetS());

		double initialOffset = 0;
		double newOffset = (initialOffset + width) * ( (cos((startTime - simulationTime)* f)- 1) / 2 );

		int roadId = position.GetTrackId();
		int laneId = position.GetLaneId();
		double s = position.GetS();

		roadmanager::Position newPosition(roadId, laneId, s, newOffset);

		(*carsPtr).setPosition(actionEntities[i], newPosition);

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
		for (size_t i = 0; i < actionEntities.size(); i++)
		{

			double newSpeed = (*carsPtr).getSpeed(actionEntities[i]) + privateAction.Longitudinal.Speed.Dynamics.rate*timeStep;
			(*carsPtr).setSpeed(actionEntities[i], newSpeed);

			if (privateAction.Longitudinal.Speed.Target.Absolute.value != NAN)
			{
				if (privateAction.Longitudinal.Speed.Target.Absolute.value >= newSpeed)
				{
					ActionCompleted = true;
					startAction = false;
				}
			}
		}
	}
}


