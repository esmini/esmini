#include "Action.hpp"

#define SMALL_NUMBER (1E-6)
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

	else if (!isnan(privateAction.Longitudinal.Speed.Dynamics.rate)) // Should be (!...Dynamics.shape.empty()) Wrong in osc
	{
		this->actionType = "speed-rate";
		this->speedRate = privateAction.Longitudinal.Speed.Dynamics.rate;
	}

	// Speed action -Step
	else if (privateAction.Longitudinal.Speed.Dynamics.shape == "step")
	{

		if (privateAction.Longitudinal.Speed.Target.Absolute.value != NAN)
		{
			this->actionType = "speed-step";
			this->speedTarget = privateAction.Longitudinal.Speed.Target.Absolute.value;
		}
	}

	// Position lane
	else if (!privateAction.Position.Lane.roadId.empty())
	{
		this->actionType = "position-lane";
	}

	// Position route
	else if (!privateAction.Position.Route.RouteRef.CatalogReference.catalogName.empty())
	{
		this->actionType = "position-route";
	}

	// Follow route
	else if (!privateAction.Routing.FollowRoute.CatalogReference.catalogName.empty())
	{
		this->actionType = "follow-route-catalog";
		for (auto &entity : actionEntities)
		{
			Car *car = this->carsPtr->getCarPtr(entity);
			for (auto &r : carsPtr->route)
			{
				// Find specified route
				if (r.getName() == privateAction.Routing.FollowRoute.CatalogReference.entryName)
				{
					std::cout << "Adding route " << r.getName() << " to car " << car->getObjectName() << std::endl;
					car->setRoute(r);
					break;
				}
			}
		}
	}

	// Meeting
	else if (!privateAction.Meeting.mode.empty())
	{
		this->actionType = "meeting";

		this->mode = privateAction.Meeting.mode;
		this->object = privateAction.Meeting.Relative.object;
		this->offsetTime = privateAction.Meeting.Relative.offsetTime;
		this->continuous = privateAction.Meeting.Relative.continuous;

		if (!privateAction.Meeting.Position.Lane.roadId.empty())
		{
			int roadId = std::stoi(privateAction.Meeting.Position.Lane.roadId);
			int lane_id = privateAction.Meeting.Position.Lane.laneId;
			double s = privateAction.Meeting.Position.Lane.s;
			double offset = privateAction.Meeting.Position.Lane.offset;

			ownTargetPos.SetLanePos(roadId, lane_id, s, offset);
		}

		if (!privateAction.Meeting.Relative.Position.Lane.roadId.empty())
		{
			int roadId = std::stoi(privateAction.Meeting.Relative.Position.Lane.roadId);
			int lane_id = privateAction.Meeting.Relative.Position.Lane.laneId;
			double s = privateAction.Meeting.Relative.Position.Lane.s;
			double offset = privateAction.Meeting.Relative.Position.Lane.offset;

			relativeTargetPos = roadmanager::Position(roadId, lane_id, s, offset);
			int a = 0;
		}		
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

	else if (actionType == "speed-step")
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

void Action::executeSpeedRate(double simulationTime, double timeStep)
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

void Action::executeSpeedStep()
{
	for (size_t i = 0; i < actionEntities.size(); i++)
	{
		(*carsPtr).setSpeed(actionEntities[i], speedTarget);
		
	}
	actionCompleted = true;
	startAction = false;
}

void Action::executePositionLane()
{
	OSCPosition position = privateAction.Position;

	int roadId = std::stoi(position.Lane.roadId);
	int laneId = position.Lane.laneId;
	double s = position.Lane.s;
	double offset = (std::isnan(position.Lane.offset)) ? 0 : position.Lane.offset;

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
	std::string routeEntryName = privateAction.Position.Route.RouteRef.CatalogReference.entryName;

	for (size_t i = 0; i < actionEntities.size(); i++)
	{
		double pathS = privateAction.Position.Route.Position.LaneCoord.pathS;
		int laneId = privateAction.Position.Route.Position.LaneCoord.laneId;

		// Find route
		roadmanager::Route * routePtr = 0;
		for (auto &r : carsPtr->route)
		{
			if (r.getName() == routeEntryName)
			{
				routePtr = &r;
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
			if (continuous == "false")
			{
				run = false;
			}

			carsPtr->setSpeed(actionEntities[0], pivotSpeed);
		}
	}
}
