#include "ScenarioGateway.hpp"


ObjectState::ObjectState(int id, std::string name, double timestamp, double x, double y, double z, double h, double p, double r, double speed)
{
	state_.posType = GW_POS_TYPE_XYH;

	setId(id);
	setName(name);
	setTimeStamp(timestamp);
	setPos(x, y, z, h, p, r, speed);
}

ObjectState::ObjectState(int id, std::string name, double timestamp, int roadId, int laneId, double laneOffset, double s, double speed)
{
	state_.posType = GW_POS_TYPE_ROAD;

	setId(id);
	setName(name);
	setTimeStamp(timestamp);
	setRoadPos(roadId, laneId, s, laneOffset, speed);
}

void ObjectState::setPos(double x, double y, double z, double h, double p, double r, double speed)
{
	state_.obj_state.base.pos.x = (float)x;
	state_.obj_state.base.pos.y = (float)y;
	state_.obj_state.base.pos.z = (float)z;
	
	state_.obj_state.base.pos.h = (float)h;
	state_.obj_state.base.pos.p = (float)p;
	state_.obj_state.base.pos.r = (float)r;

	// Divide into X, Y components according to heading
	setVelocity(speed);
}

void ObjectState::setRoadPos(int roadId, int laneId, double s, double laneOffset, double speed)
{
	state_.road_pos.roadId = roadId;
	state_.road_pos.laneId = laneId;
	state_.road_pos.laneOffset = (float)laneOffset;
	state_.road_pos.roadS = (float)laneOffset;

	// Divide into X, Y components according to heading
	setVelocity(speed);
}

void ObjectState::setVelocity(double speed)
{
	// Divide into X, Y components according to heading
	state_.obj_state.ext.speed.x = (float)(speed * cos(state_.obj_state.base.pos.h));
	state_.obj_state.ext.speed.y = (float)(speed * sin(state_.obj_state.base.pos.h));
	state_.obj_state.ext.speed.z = 0.0f;
}

void ObjectState::setName(std::string name)
{
//	name.copy(state_.obj_state.base.name, RDB_SIZE_OBJECT_NAME);	// compiler warning...
	strncpy_s(state_.obj_state.base.name, RDB_SIZE_OBJECT_NAME, name.c_str(), name.size());  // Accepted by VS/Windows
}

void ObjectState::Print()
{
	printf("state: \n\tid %d\n\ttime %.2f\n\tx %.2f\n\ty %.2f\n\th %.2f\n\tvel (%.2f, %.2f, %.2f)\n",
		state_.obj_state.base.id,
		state_.header.simTime,
		state_.obj_state.base.pos.x,
		state_.obj_state.base.pos.y,
		state_.obj_state.base.pos.h,
		state_.obj_state.ext.speed.x,
		state_.obj_state.ext.speed.y, 
		state_.obj_state.ext.speed.z
	);
}

double ObjectState::convertVelocityToSpeed(double vel_x, double vel_y, double rot_h)
{
	double speed;
	
	// Calculate magnitude of speed
	speed = sqrt(vel_x * vel_x + vel_y * vel_y);

	// Find out direction of speed, going forward or backwards? Compare with heading
	double rotatedVelX = vel_x * cos(-rot_h) - vel_y * sin(-rot_h);
	double rotatedVelY = vel_x * sin(-rot_h) + vel_y * cos(-rot_h);

	int sign = rotatedVelX < 0 ? -1 : 1;

	speed *= sign;

	return speed; 
}

// ScenarioGateway
ScenarioGateway::ScenarioGateway()
{
}

ObjectState *ScenarioGateway::getObjectStateById(int id)
{
	for (auto &o : objectState_)
	{
		if (o.getId() == id)
		{
			return &o;
		}
	}

	// Indicate not found by returning NULL
	return 0;
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
			o = objectState;			
			//printf("ScenarioGateway::reportObject Updating %s state: (%d, %.2f)\n", o.getName().c_str(), o.getId(), o.getTimeStamp());
			//o.Print();
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
