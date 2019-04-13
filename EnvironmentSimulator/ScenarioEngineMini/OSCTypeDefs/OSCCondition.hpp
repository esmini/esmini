#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include "OSCCommon.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "OSCPosition.hpp"

namespace scenarioengine
{

	// Forward declaration 
	class Story;
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
		bool evaluated_;
		bool last_result_;  // result from last evaluation
		ConditionEdge edge_;

		OSCCondition(ConditionType base_type) : base_type_(base_type), evaluated_(false), last_result_(false), edge_(ConditionEdge::ANY) {}

		virtual bool Evaluate(Story *story, double sim_time) = 0;
		bool CheckEdge(bool new_value, bool old_value, OSCCondition::ConditionEdge edge);
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
			DISTANCE,
			RELATIVE_DISTANCE,
			REACH_POSITION,
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

		TrigByTimeHeadway() : TrigByEntity(TrigByEntity::EntityConditionType::TIME_HEADWAY) {}

		bool Evaluate(Story *story, double sim_time);
	};

	class TrigByReachPosition : public TrigByEntity
	{
	public:
		OSCPosition *position_;
		double tolerance_;

		TrigByReachPosition() : TrigByEntity(TrigByEntity::EntityConditionType::REACH_POSITION) {}

		bool Evaluate(Story *story, double sim_time);
	};

	class TrigByDistance : public TrigByEntity
	{
	public:
		OSCPosition *position_;
		double value_;
		bool freespace_;
		bool along_route_;
		Rule rule_;

		TrigByDistance() : TrigByEntity(TrigByEntity::EntityConditionType::DISTANCE) {}

		bool Evaluate(Story *story, double sim_time);
	};

	class TrigByRelativeDistance : public TrigByEntity
	{
	public:
		typedef enum
		{
			LONGITUDINAL,
			LATERAL,
			INTERIAL
		} RelativeDistanceType;

		Object *object_;
		double value_;
		bool freespace_;
		RelativeDistanceType type_;
		Rule rule_;

		TrigByRelativeDistance() : TrigByEntity(TrigByEntity::EntityConditionType::RELATIVE_DISTANCE) {}

		bool Evaluate(Story *story, double sim_time);
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

		bool Evaluate(Story *story, double sim_time);
	};

	class TrigAtStart : public TrigByState
	{
	public:
		StoryElementType element_type_;

		TrigAtStart() : TrigByState(TrigByState::Type::AT_START) {}

		bool Evaluate(Story *story, double sim_time);
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

		bool Evaluate(Story *story, double sim_time);
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

		bool Evaluate(Story *story, double sim_time);
	};

	class TrigBySimulationTime : public TrigByValue
	{
	public:
		double value_;

		TrigBySimulationTime() : TrigByValue(TrigByValue::Type::TIME_OF_DAY) {}

		bool Evaluate(Story *story, double sim_time);
	};

}
