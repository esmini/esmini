#include "Action.hpp"


Action::Action(OSCPrivateAction &privateAction, Cars &cars, std::vector<int> storyId, std::vector<std::string> &actionEntities)
{
	this->privateAction = privateAction;
	carsPtr = &cars;
	this->actionEntities = actionEntities;
	this->storyId = storyId;

	actionCompleted = false;
	startAction = false;
	firstRun = true;

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
			this->laneChange = true;
		}
	}
	else if (privateAction.Lateral.LaneOffset.Dynamics.shape == "sinusoidal")
	{
		if (!isnan(privateAction.Lateral.LaneOffset.Dynamics.duration))
		{
			this->actionType = "sinusoidal-time";
			this->time = privateAction.Lateral.LaneOffset.Dynamics.duration;
			this->targetObject = privateAction.Lateral.LaneOffset.Target.Relative.object;
			this->targetValue = privateAction.Lateral.LaneOffset.Target.Relative.value;
			this->f = 3.1415 / time;
			this->laneChange = false;
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
	if (firstRun)
	{
		firstRun = false;

		// Target entities
		roadmanager::Position tObjectPosition = (*carsPtr).getPosition(targetObject);
		roadmanager::Road *tObjectRoad = tObjectPosition.GetOpenDrive()->GetRoadById(tObjectPosition.GetTrackId());
		tT = tObjectRoad->GetCenterOffset(tObjectPosition.GetS(), tObjectPosition.GetLaneId() + targetValue);

		// Action entities
		aT.resize(actionEntities.size());

		for (size_t i = 0; i < actionEntities.size(); i++)
		{
			roadmanager::Position aPosition = (*carsPtr).getPosition(actionEntities[i]);
			roadmanager::Road *aRoad = aPosition.GetOpenDrive()->GetRoadById(aPosition.GetTrackId());
			aT[i] = aRoad->GetCenterOffset(aPosition.GetS(), aPosition.GetLaneId()) - aPosition.GetOffset();
		}
	}

	for (size_t i = 0; i < actionEntities.size(); i++)
	{

		double newT = (tT - aT[i]) * ((cos((startTime - simulationTime)* f) - 1) / 2) - aT[i];

		// Create new position
		int roadId = (*carsPtr).getPosition(actionEntities[i]).GetTrackId();
		int laneId = (*carsPtr).getPosition(actionEntities[i]).GetLaneId();
		double s = (*carsPtr).getPosition(actionEntities[i]).GetS();

		roadmanager::Position newPosition;

		if (laneChange)
		{
			newPosition.SetTrackPos(roadId, s, newT);
		}
		else
		{
			roadmanager::Position aPosition = (*carsPtr).getPosition(actionEntities[i]);
			roadmanager::Road *aRoad = aPosition.GetOpenDrive()->GetRoadById(roadId);
			roadmanager::LaneSection *aLanesection = aRoad->GetLaneSectionByS(s);
			double tmp = aLanesection->GetCenterOffset(s, laneId);

			newPosition.SetLanePos(roadId, laneId, s, tmp+newT);
		}

		double x = newPosition.GetX();
		double y = newPosition.GetY();
		double z = newPosition.GetZ();

		double h = newPosition.GetH() + 0.1*sin((startTime - simulationTime)*f);
		double p = newPosition.GetP();
		double r = newPosition.GetR();

		newPosition.SetInertiaPos(x, y, z, h, p, r);

		(*carsPtr).setPosition(actionEntities[i], newPosition);
	}
	
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


