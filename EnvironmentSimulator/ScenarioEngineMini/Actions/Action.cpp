#include "Action.hpp"
#include "CommonMini.hpp"

#define DISTANCE_TOLERANCE (0.5)  // meter
#define IS_ZERO(x) (x < SMALL_NUMBER && x > -SMALL_NUMBER)

Action::Action(OSCPrivateAction &privateAction, Cars &cars, std::vector<int> storyId, std::vector<std::string> &actionEntities)
{
	this->privateAction = privateAction;
	carsPtr = &cars;
	this->actionEntities = actionEntities;
	this->storyId = storyId;

	actionCompleted = false;
	startAction = false;
	firstRun = true;

	if (false)
	{
		actionName = "Whats this";
	}

	identifyActionType(privateAction);	
}

std::string Action::getActionType()
{
	return actionType;
}

std::string Action::getActionName()
{
	return actionName;
}

void Action::identifyActionType(OSCPrivateAction privateAction)
{
	if (privateAction.laneChange_)
	{
		if (privateAction.laneChange_)
		{
			this->actionType = "sinusoidal-time";
		}
		this->time = privateAction.laneChange_->Dynamics.time;
		if (privateAction.laneChange_->Target.relative_)
		{
			this->targetObject = privateAction.laneChange_->Target.relative_->object;
			this->targetValue = privateAction.laneChange_->Target.relative_->value;
		}
		else if (privateAction.laneChange_->Target.absolute_)
		{
			this->targetValue = privateAction.laneChange_->Target.absolute_->value;
		}
		this->f = 3.1415 / time;
		this->laneChange = true;
	}
	else if (privateAction.laneOffset_)
	{
		if (privateAction.laneOffset_->Dynamics.shape == std::string("sinusoidal"))
		{
			this->actionType = "sinusoidal-time";
		}
		this->time = privateAction.laneOffset_->Dynamics.duration;
		if (privateAction.laneOffset_->Target.relative_)
		{
			this->targetObject = privateAction.laneOffset_->Target.relative_->object;
			this->targetValue = privateAction.laneOffset_->Target.relative_->value;
		}
		else if (privateAction.laneOffset_->Target.absolute_)
		{
			this->targetValue = privateAction.laneOffset_->Target.absolute_->value;
		}
		this->f = 3.1415 / time;
		this->laneChange = false;
	}

	else if (privateAction.speed_) // Should be (!...Dynamics.shape.empty()) Wrong in osc
	{
		if (fabs(privateAction.speed_->dynamics_->rate) > SMALL_NUMBER)
		{
			this->actionType = "speed-rate";
			this->speedRate = privateAction.speed_->dynamics_->rate;
		}
		else if (privateAction.speed_->dynamics_->shape == "step")
		{
			if (privateAction.speed_->target_->absolute_)
			{
				this->actionType = "speed-step-absolute";
				this->speedTarget = privateAction.speed_->target_->absolute_->value;
			}
			else if (privateAction.speed_->target_->relative_)
			{
				this->actionType = "speed-step-relative";
				this->speedTarget = privateAction.speed_->target_->relative_->value;
				this->object = privateAction.speed_->target_->relative_->object;
				this->valueType = privateAction.speed_->target_->relative_->valueType;
				this->continuous = privateAction.speed_->target_->relative_->continuous;
				this->valueType = privateAction.speed_->target_->relative_->valueType;
			}
		}
	}

	// Position lane
	else if (privateAction.position_ && privateAction.position_->lane_)
	{
		this->actionType = "position-lane";
	}

	// Position route
	else if (privateAction.position_ && privateAction.position_->route_)
	{
		this->actionType = "position-route";
	}

	// Follow route
	else if (privateAction.routing_)
	{
		this->actionType = "follow-route-catalog";
		for (size_t i=0; i<actionEntities.size(); i++)
		{
			Car *car = this->carsPtr->getCarPtr(actionEntities[i]);
			for (size_t j=0; j<carsPtr->route.size(); j++)
			{
				// Find specified route
				if (carsPtr->route[j].getName() == privateAction.routing_->FollowRoute.CatalogReference.entryName)
				{
					LOG("Adding route %s to car %s", carsPtr->route[j].getName().c_str(), car->getObjectName().c_str());
					car->setRoute(carsPtr->route[j]);
					break;
				}
			}
		}
	}

	// Meeting
	else if (privateAction.meeting_)
	{
		this->actionType = "meeting";

		this->mode = privateAction.meeting_->mode;
		this->object = privateAction.meeting_->relative_->object;
		this->offsetTime = privateAction.meeting_->relative_->offsetTime;
		this->continuous = privateAction.meeting_->relative_->continuous;

		int roadId = privateAction.meeting_->Position.lane_->roadId;
		int lane_id = privateAction.meeting_->Position.lane_->laneId;
		double s = privateAction.meeting_->Position.lane_->s;
		double offset = privateAction.meeting_->Position.lane_->offset;

		ownTargetPos.SetLanePos(roadId, lane_id, s, offset);

		roadId = privateAction.meeting_->relative_->Position.lane_->roadId;
		lane_id = privateAction.meeting_->relative_->Position.lane_->laneId;
		s = privateAction.meeting_->relative_->Position.lane_->s;
		offset = privateAction.meeting_->relative_->Position.lane_->offset;

		relativeTargetPos = roadmanager::Position(roadId, lane_id, s, offset);
		int a = 0;
	}
}

int Action::sign(int value)
{
	if (value < 0)
	{
		return -1;
	}
	else
	{
		return 1;
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

	else if (actionType == "speed-rate")
	{
		executeSpeedRate(simulationTime, timeStep);
	}

	else if (actionType == "speed-step-absolute" || actionType == "speed-step-relative")
	{
		executeSpeedStep();
	}

	else if (actionType == "position-lane")
	{
		executePositionLane();
	}

	else if (actionType == "position-route")
	{
		executePositionRoute();
	}

	else if (actionType == "follow-route-catalog")
	{
		executeFollowRoute();
	}

	else if (actionType == "meeting")
	{
		executeMeeting();
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
		if (laneChange)
		{
			tT = tObjectRoad->GetCenterOffset(tObjectPosition.GetS(), tObjectPosition.GetLaneId() + (int)targetValue);
		}
		else
		{
			tT = tObjectRoad->GetCenterOffset(tObjectPosition.GetS(), tObjectPosition.GetLaneId()) + targetValue;
		}

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

		newPosition.SetInertiaPos(x, y, z, h, p, r, false);

		(*carsPtr).setPosition(actionEntities[i], newPosition);
	}
	
	// Should use the target position instead of time to decide when action is completed
	if (simulationTime >= startTime + time)
	{
		actionCompleted = true;
		startAction = false;
	}
}

void Action::executeSpeedRate(double simulationTime, double timeStep)
{
	for (size_t i = 0; i < actionEntities.size(); i++)
	{

		double newSpeed = (*carsPtr).getSpeed(actionEntities[i]) + speedRate*timeStep;
		(*carsPtr).setSpeed(actionEntities[i], newSpeed);

		if (privateAction.speed_->target_->absolute_->value >= newSpeed)
		{
			actionCompleted = true;
			startAction = false;
		}
	}
}

void Action::executeSpeedStep()
{
	for (size_t i = 0; i < actionEntities.size(); i++)
	{
		double targetSpeed = speedTarget;
		
		if (actionType == "speed-step-relative")
		{
			if (valueType == "delta")
			{
				targetSpeed += carsPtr->getCarPtr(object)->getSpeed();
			}
			else
			{
				LOG("valueType %s not supported yet", valueType);
			}
		}
		else if(actionType == "speed-step-absolute")
		{
			actionCompleted = true;
			startAction = false;
		}
		
		(*carsPtr).setSpeed(actionEntities[i], targetSpeed);		
	}
}

void Action::executePositionLane()
{
	int roadId = privateAction.position_->lane_->roadId;
	int laneId = privateAction.position_->lane_->laneId;
	double s = privateAction.position_->lane_->s;
	double offset = privateAction.position_->lane_->offset;

	roadmanager::Position pos(roadId, laneId, s, offset);

	for (size_t i = 0; i < actionEntities.size(); i++)
	{
		(*carsPtr).setPosition(actionEntities[i], pos);
	}

	actionCompleted = true;
	startAction = false;

}

void Action::executePositionRoute()
{
	std::string routeEntryName = privateAction.position_->route_->RouteRef.CatalogReference.entryName;

	for (size_t i = 0; i < actionEntities.size(); i++)
	{
		double pathS = privateAction.position_->route_->Position.LaneCoord.pathS;
		int laneId = privateAction.position_->route_->Position.LaneCoord.laneId;

		// Find route
		roadmanager::Route * routePtr = 0;
		for (size_t j=0; j<carsPtr->route.size(); j++)
		{
			if (carsPtr->route[j].getName() == routeEntryName)
			{
				routePtr = &carsPtr->route[j];
				break;
			}
		}

		routePtr->Set(pathS, laneId, 0);
		routePtr->GetPosition(carsPtr->getCarPtr(actionEntities[i])->getPositionPtr());
	}
	actionCompleted = true;
	startAction = false;
}

void Action::executeFollowRoute()
{
	for (size_t i = 0; i < actionEntities.size(); i++)
	{
		carsPtr->setFollowRoute(actionEntities[i], true);
	}

	actionCompleted = true;
	startAction = false;
}

void Action::executeMeeting()
{
	// std::string mode;     -> is not implemented yet

	// Make sure that all other actions have been initialized
	if (firstRun)
	{
		firstRun = false;
	}
	else
	{
		bool run = true;

		// Calculate straight distance, not along road/route. To be improved.
		Car *pivotCar = carsPtr->getCarPtr(actionEntities[0]);
		Car *targetCar = carsPtr->getCarPtr(object);

		roadmanager::Position *pivotCarPos = pivotCar->getPositionPtr();
		roadmanager::Position *targetCarPos = targetCar->getPositionPtr();

		double pivotDist = pivotCarPos->getRelativeDistance(ownTargetPos);
		double targetDist = targetCarPos->getRelativeDistance(relativeTargetPos);

		double targetTimeToDest = INFINITY;
		double targetDeltaTime = INFINITY;

		if (targetCar->getSpeed() > SMALL_NUMBER)
		{
			targetTimeToDest = targetDist / targetCar->getSpeed();
		}

		double pivotSpeed = pivotDist / targetTimeToDest;

#if 0
		printf("pivot:  Dist %8.2f OldSpeed   %6.2f NewSpeed %6.2f\n", pivotDist, pivotCar->getSpeed(), pivotSpeed);
		printf("target: Dist %8.2f TimeToDest %6.2f NewSpeed %6.2f\n", targetDist, targetTimeToDest, targetCar->getSpeed());
#endif

		if (targetDist < DISTANCE_TOLERANCE)
		{
			actionCompleted = true;
			startAction = false;
		}
		else if (run)
		{
			if (continuous == false)
			{
				run = false;
			}

			carsPtr->setSpeed(actionEntities[0], pivotSpeed);
		}
	}
}
