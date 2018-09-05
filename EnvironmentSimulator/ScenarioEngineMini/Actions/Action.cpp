#include "Action.hpp"


Action::Action(OSCPrivateAction &privateAction, Cars &cars, std::vector<int> storyId, std::vector<std::string> &actionEntities)
{
	this->privateAction = privateAction;
	carsPtr = &cars;
	this->actionEntities = actionEntities;
	this->storyId = storyId;

	actionCompleted = false;
	startAction = false;
	
	identifyActionType(privateAction);

}

void Action::identifyActionType(OSCPrivateAction privateAction)
{
	if (privateAction.Lateral.LaneChange.Dynamics.shape == "sinusoidal")
	{
		if (!isnan(privateAction.Lateral.LaneChange.Dynamics.time))
		{
			this->actionType = "sinusoidal-time";
			this->time = privateAction.Lateral.LaneChange.Dynamics.time;
			this->targetObject = privateAction.Lateral.LaneChange.Target.Relative.object;
			this->targetValue = privateAction.Lateral.LaneChange.Target.Relative.value;
			this->f = 3.1415 / time;
		}
	}

	if (!isnan(privateAction.Longitudinal.Speed.Dynamics.rate)) // Should be (!...Dynamics.shape.empty()) Wrong in osc
	{
		this->actionType = "speed";
		this->speedRate = privateAction.Longitudinal.Speed.Dynamics.rate;
	}
}

void Action::setStartTime(double simulationTime)
{
	startTime = simulationTime;
}

void Action::setStartAction()
{
	startAction = true;
}

bool Action::getStartAction()
{
	return startAction;
}

bool Action::getActionCompleted()
{
	return actionCompleted;
}

std::vector<int> Action::getStoryId()
{
	return storyId;
}

void Action::ExecuteAction(double simulationTime, double timeStep) {

	if (actionType == "sinusoidal-time")
	{
		executeSinusoidal(simulationTime);
	}

	if (actionType == "speed")
	{
		executeSpeed(simulationTime, timeStep);
	}
}

void Action::executeSinusoidal(double simulationTime)
{

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

		// Create new position
		int roadId = position.GetTrackId();
		int laneId = position.GetLaneId();
		double s = position.GetS();

		roadmanager::Position newPosition(roadId, laneId, s, newOffset);

		double x = newPosition.GetX();
		double y = newPosition.GetY();
		double z = newPosition.GetZ();

		double h = newPosition.GetH() + 0.1*sin((startTime - simulationTime)*f);
		double p = newPosition.GetP();
		double r = newPosition.GetR();

		newPosition.SetInertiaPos(x, y, z, h, p, r);

		(*carsPtr).setPosition(actionEntities[i], newPosition);
	}
		
	// Could switch lane after time/2 and swap sign of the offset

	// Should use the target position instead of time to decide when action is completed
	if (simulationTime >= startTime + time)
	{
		actionCompleted = true;
		startAction = false;
	}
}

void Action::executeSpeed(double simulationTime, double timeStep)
{
	if (privateAction.Longitudinal.Speed.Dynamics.rate != NAN)
	{
		for (size_t i = 0; i < actionEntities.size(); i++)
		{

			double newSpeed = (*carsPtr).getSpeed(actionEntities[i]) + speedRate*timeStep;
			(*carsPtr).setSpeed(actionEntities[i], newSpeed);

			if (privateAction.Longitudinal.Speed.Target.Absolute.value != NAN)
			{
				if (privateAction.Longitudinal.Speed.Target.Absolute.value >= newSpeed)
				{
					actionCompleted = true;
					startAction = false;
				}
			}
		}
	}
}


