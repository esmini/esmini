#include "ScenarioGateway.hpp"


ObjectState::ObjectState(int id, std::string name, double timestamp, double x, double y, double h, double speed)
{
	setId(id);
	setName(name);
	setTimeStamp(timestamp);
	setPos(x, y, h, speed);
}

ObjectState::ObjectState(int id, std::string name, double timestamp, int roadId, int laneId, double laneOffset, double s, double speed)
{
	setId(id);
	setName(name);
	setTimeStamp(timestamp);
	setRoadPos(roadId, laneId, s, laneOffset, speed);
}

void ObjectState::setPos(double x, double y, double h, double speed)
{
	state_.obj_state.base.pos.x = (float)x;
	state_.obj_state.base.pos.y = (float)y;
	state_.obj_state.base.pos.z = 0.0f;
	
	state_.obj_state.base.pos.h = (float)h;
	state_.obj_state.base.pos.p = 0.0f;
	state_.obj_state.base.pos.r = 0.0f;

	// Divide into X, Y components according to heading
	setVelocity(speed);

	calculateRoadPos();
}

void ObjectState::setRoadPos(int roadId, int laneId, double s, double laneOffset, double speed)
{
	state_.road_pos.roadId = roadId;
	state_.road_pos.laneId = laneId;
	state_.road_pos.laneOffset = (float)laneOffset;
	state_.road_pos.roadS = (float)laneOffset;

	setVelocity(speed);

	calculateXYH();
}

void ObjectState::setVelocity(double speed)
{
	// Divide into X, Y components according to heading
	state_.obj_state.ext.speed.x = (float)(speed * cos(state_.obj_state.base.pos.h));
	state_.obj_state.ext.speed.y = (float)(speed * sin(state_.obj_state.base.pos.h));
	state_.obj_state.ext.speed.z = 0.0f;
}

void ObjectState::calculateXYH()
{
	roadmanager::Position pos;

	pos.SetLanePos(state_.road_pos.roadId, state_.road_pos.laneId, state_.road_pos.roadS, state_.road_pos.laneOffset);

	state_.obj_state.base.pos.x = (float)pos.GetX();
	state_.obj_state.base.pos.y = (float)pos.GetY();
	state_.obj_state.base.pos.z = (float)pos.GetZ();
}

void ObjectState::calculateRoadPos()
{
	roadmanager::Position pos;

	pos.SetXYH(state_.obj_state.base.pos.x, state_.obj_state.base.pos.y, state_.obj_state.base.pos.h);

	state_.road_pos.roadId = pos.GetTrackId();
	state_.road_pos.laneId = pos.GetLaneId();
	state_.road_pos.pathS = (float)pos.GetS();
}

void ObjectState::setName(std::string name)
{
//	name.copy(state_.obj_state.base.name, RDB_SIZE_OBJECT_NAME);	// compiler warning...
	strncpy_s(state_.obj_state.base.name, RDB_SIZE_OBJECT_NAME, name.c_str(), name.size());  // Accepted by VS/Windows
}

// ScenarioGateway
ScenarioGateway::ScenarioGateway()
{
}

void ScenarioGateway::reportObject(ObjectState objectState)
{
	bool found = false;

	// Check whether the object is already present in the list of active objects
	for (auto &o : objectState_)
	{
		if (o.getId() == objectState.getId())
		{
			found = true;
			
			// Update state
			printf("Updating %s state: (%d, %.2f)\n", o.getName().c_str(), o.getId(), o.getTimeStamp());
			o = objectState;			
			break;
		}
	}

	if (!found)
	{
		// Add object
		printf("Adding %s state: (%d, %.2f)\n", objectState.getName().c_str(), objectState.getId(), objectState.getTimeStamp());
		objectState_.push_back(objectState);
	}
}
