#pragma once
#include "roadmanager.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#define NAME_LEN 32

struct ObjectStateStruct
{
	int id;
	float timeStamp;
	char name[NAME_LEN];
	roadmanager::Position pos;
	float speed;
};

class ObjectState
{
public:
	ObjectState();
	ObjectState(int id, std::string name, double timestamp, double x, double y, double z, double h, double p, double r, double speed);
	ObjectState(int id, std::string name, double timestamp, int roadId, int laneId, double laneOffset, double s, double speed);

	ObjectStateStruct getStruct() { return state_; }

	void Print();

	ObjectStateStruct state_;

private:

	friend class ScemarioGateway;
};


class ScenarioGateway
{
public:
	
	ScenarioGateway();
	~ScenarioGateway();

	void reportObject(ObjectState objectState);
	int getNumberOfObjects() { return (int)objectState_.size(); }
	ObjectState getObjectStateByIdx(int idx) { return *objectState_[idx]; }
	ObjectState *getObjectStatePtrByIdx(int idx) { return objectState_[idx]; }
	int getObjectStateById(int idx, ObjectState &objState);

private:
	std::vector<ObjectState*> objectState_;
};

