#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include "OSCCommon.hpp"
#include "CommonMini.hpp"

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

	OSCCondition(ConditionType base_type) : base_type_(base_type) {}
};

class TrigByEntity : public OSCCondition
{
public:
	struct Entity
	{
		std::string name_;
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
	std::string entity_name_;
	double value_;
	bool freespace_;
	bool along_route_;
	Rule rule_;

	TrigByTimeHeadway() : TrigByEntity(TrigByEntity::EntityConditionType::TIME_HEADWAY) {}
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
};

class TrigAtStart : public TrigByState
{
public:
	StoryElementType element_type_;

	TrigAtStart() : TrigByState(TrigByState::Type::AT_START) {}
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
};

class TrigBySimulationTime : public TrigByValue
{
public:
	double value_;

	TrigBySimulationTime() : TrigByValue(TrigByValue::Type::TIME_OF_DAY) {}
};

