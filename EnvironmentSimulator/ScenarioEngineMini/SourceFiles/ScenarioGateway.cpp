#include "ScenarioGateway.hpp"

ObjectState::ObjectState()
{
	memset(&state_, 0, sizeof(ObjectState));
	state_.id = -1;
}

ObjectState::ObjectState(int id, std::string name, double timestamp, double x, double y, double z, double h, double p, double r, double speed)
{
	memset(&state_, 0, sizeof(ObjectStateStruct));

	state_.id = id;
	state_.timeStamp = (float)timestamp;
	strncpy(state_.name, name.c_str(), NAME_LEN);
	state_.pos.SetInertiaPos(x, y, z, h, p, r);
	state_.speed = (float)speed;
}

ObjectState::ObjectState(int id, std::string name, double timestamp, int roadId, int laneId, double laneOffset, double s, double speed)
{
	memset(&state_, 0, sizeof(ObjectStateStruct));

	state_.id = id;
	state_.timeStamp = (float)timestamp;
	strncpy(state_.name, name.c_str(), NAME_LEN);
	state_.pos.SetLanePos(roadId, laneId, s, laneOffset);
	state_.speed = (float)speed;
}

void ObjectState::Print()
{
	printf("state: \n\tid %d\n\tname %s\n\ttime %.2f\n\tx %.2f\n\ty %.2f\n\th %.2f\n\tspeed %.2f\n",
		state_.id,
		state_.name,
		state_.timeStamp,
		state_.pos.GetX(),
		state_.pos.GetY(),
		state_.pos.GetZ(),
		state_.speed
	);
}

// ScenarioGateway
ScenarioGateway::ScenarioGateway()
{
	objectState_.clear();
}

ScenarioGateway::~ScenarioGateway()
{
	for (auto o : objectState_)
	{
		delete o;
	}
	objectState_.clear();
}


int ScenarioGateway::getObjectStateById(int id, ObjectState &objectState)
{
	for (auto o : objectState_)
	{
		if (o->state_.id == id)
		{
			objectState = *o;
			return 0;
		}
	}

	// Indicate not found by returning non zero
	return -1;

}

void ScenarioGateway::reportObject(ObjectState objectState)
{
	bool found = false;
	
	// Check whether the object is already present in the list of active objects
	for (auto o : objectState_)
	{
		if (o->state_.id == objectState.state_.id)
		{
			found = true;
			
			// Update state
			*o = objectState;	
			break;
		}
	}

	if (!found)
	{
		// Add object
		printf("Adding %s state: (%d, %.2f)\n", objectState.state_.name, objectState.state_.id, objectState.state_.timeStamp);
		ObjectState *os = new ObjectState;
		*os = objectState;
		objectState_.push_back(os);
	}
}
