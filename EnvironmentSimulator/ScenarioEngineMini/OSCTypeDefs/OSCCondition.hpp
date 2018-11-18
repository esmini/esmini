#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include "OSCCommon.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"

// Forward declaration 
class Act;

class OSCCondition
{
public:
	
	typedef enum
	{
		BY_ENTITY,
		BY_STATE,
		BY_VALUE
	} ConditionType;

	typedef enum
	{
		RISING,
		FALLING,
		ANY,
		UNDEFINED
	} ConditionEdge;

	ConditionType base_type_;
	std::string name_;
	double delay_;
	ConditionEdge edge_; 
	bool evaluated_;

	OSCCondition(ConditionType base_type) : base_type_(base_type), evaluated_(false) {}
	
	virtual bool Evaluate(Act *act, double sim_time) = 0;
	bool CheckEdge(double a, double b, OSCCondition::ConditionEdge edge);
};

class TrigByEntity : public OSCCondition
{
public:
	struct Entity
	{
		Object *object_;
	};

	typedef enum
	{
		ANY,
		ALL
	} TriggeringEntitiesRule;

	struct TriggeringEntities
	{
		std::vector<Entity> entity_;
		TriggeringEntitiesRule rule_;
	};

	typedef enum
	{
		TIME_HEADWAY,
		// not complete at all
	} EntityConditionType;

	TriggeringEntitiesRule triggering_entity_rule_;
	TriggeringEntities triggering_entities_;
	EntityConditionType type_;
	
	TrigByEntity(EntityConditionType type) : OSCCondition(OSCCondition::ConditionType::BY_ENTITY), type_(type) {}

	void Print()
	{
		LOG("");
	}
};

class TrigByTimeHeadway : public TrigByEntity
{
public:
	Object *object_;
	double value_;
	bool freespace_;
	bool along_route_;
	Rule rule_;
	double headway_time_last_value_;


	TrigByTimeHeadway() : TrigByEntity(TrigByEntity::EntityConditionType::TIME_HEADWAY) {}

	bool Evaluate(Act *act, double sim_time);
};

class TrigByState : public OSCCondition
{
public:
	typedef enum
	{
		AT_START,
		AFTER_TERMINATION,
	} Type;

	typedef enum
	{
		ACT,
		SCENE,
		MANEUVER,
		EVENT,
		ACTION,
		UNDEFINED
	} StoryElementType;

	Type type_;
	std::string element_name_;

	TrigByState(Type type) : OSCCondition(BY_STATE), type_(type) {}

	bool Evaluate(Act *act, double sim_time);
};

class TrigAtStart : public TrigByState
{
public:
	StoryElementType element_type_;

	TrigAtStart() : TrigByState(TrigByState::Type::AT_START) {}

	bool Evaluate(Act *act, double sim_time);
};

class TrigAfterTermination : public TrigByState
{
public:
	typedef enum
	{
		END,
		CANCEL,
		ANY,
		UNDEFINED
	} AfterTerminationRule;

	AfterTerminationRule rule_;
	StoryElementType element_type_;

	TrigAfterTermination() : TrigByState(TrigByState::Type::AFTER_TERMINATION) {}

	bool Evaluate(Act *act, double sim_time);
};

class TrigByValue : public OSCCondition
{
public:
	typedef enum
	{
		PARAMETER,
		TIME_OF_DAY,
		SIMULATION_TIME,
		UNDEFINED
	} Type;

	Type type_;
	Rule rule_;

	TrigByValue(Type type) : OSCCondition(BY_VALUE), type_(type) {}

	bool Evaluate(Act *act, double sim_time);
};

class TrigBySimulationTime : public TrigByValue
{
public:
	double value_;

	TrigBySimulationTime() : TrigByValue(TrigByValue::Type::TIME_OF_DAY) {}

	bool Evaluate(Act *act, double sim_time);
};

